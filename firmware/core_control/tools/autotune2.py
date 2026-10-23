#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
autotune2.py - 逐电机精密整定(速度环), 针对 M1/M2 这类需要"特别精准"的场合。

与 autotune.py 的区别(这些正是精度卡点):
  1. SET_CTRL 按电机 id 下发, 不再广播 0xFF;
  2. 开环多点辨识(默认 6 档占空比) + 最小二乘 -> K / u0 / R², 不再两点硬算;
  3. 稳态判据采样(位置差分稳定 0.25s 窗口), 不再固定 1.2s 抢跑;
  4. 稳态速度用 int32 编码器位置差分计算(分辨率 0.04%), 不用 int16 rpm;
  5. tau 用"死区+一阶+固件测速低通"级联模型做网格搜索拟合, 不再被低通带偏;
  6. 闭环验收扫多个目标转速, 输出 稳态误差/纹波/过冲/调节时间, 不达标自动重试;
  7. 结果落 JSON + 生成 per-motor 的 C 初始化片段;
  8. any 异常/中断都会刹车收尾;
  9. --dry-run 用与固件同构的仿真对象跑通全流程, 不接硬件也能验证算法.

用法:
    python autotune2.py COM7 --motor 0            # 只标 M1
    python autotune2.py COM7 --motor 0 1          # 依次标 M1/M2
    python autotune2.py COM7 --motor 0 --attempts 1 --no-reverse
    python autotune2.py --dry-run                 # 离线自测(合成模型)

依赖: pyserial (matplotlib 不需要)
"""

import argparse
import json
import math
import os
import random
import shutil
import statistics
import struct
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_script import build, read_frame          # 复用帧协议(crc8/组帧/收帧)

# ---------------------------------------------------------------- 协议常量
BRAKE, SET_SPEED, SET_DUTY, SET_CTRL, GET_TEL, GET_BAT = 0x03, 0x01, 0x05, 0x06, 0x10, 0x11
ACK, TEL, BAT = 0x80, 0x81, 0x82

# ---------------------------------------------------------------- 固件常量
ENC_CPR = 1320.0          # bsp_motor.h: 输出轴每圈计数
DUTY_MAX = 1000           # motor_service.c: 占空比千分比满幅
LPF_ALPHA = 0.35          # motor_service.c: MOTOR_SPEED_LPF_ALPHA
CTRL_DT = 0.010           # motor_task.c: 控制周期 10ms


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def sgn(v):
    return (v > 0) - (v < 0)


# ================================================================ 通道抽象
class HostChannel:
    """真实串口通道。遥测为请求-应答, 100Hz(受 comm_task 10ms 周期限制)。"""

    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.seq = 0
        self.info = {'port': port}
        self.rtt_ms = self._measure_rtt()

    def now(self):
        return time.perf_counter()

    def sleep(self, s):
        time.sleep(s)

    def _send(self, cmd, data=b''):
        self.seq = (self.seq + 1) & 0xFF
        self.ser.write(build(cmd, self.seq, data))

    def _wait(self, want, timeout):
        deadline = self.now() + timeout
        while self.now() < deadline:
            f = read_frame(self.ser, 0.03)
            if f is None:
                continue
            if want(f):
                return f
        return None

    def _measure_rtt(self):
        t0 = time.perf_counter()
        self._send(0x00)                       # PING
        f = self._wait(lambda fr: fr[3] == ACK, 0.5)
        return (time.perf_counter() - t0) * 1000.0 if f else None

    def ping(self):
        self._send(0x00)
        return self._wait(lambda fr: fr[3] == ACK, 0.5) is not None

    def brake(self, m):
        self._send(BRAKE, bytes([m]))

    def set_duty(self, m, duty):
        self._send(SET_DUTY, bytes([m]) + struct.pack('<h', int(round(duty))))

    def set_speed(self, m, rpm):
        self._send(SET_SPEED, bytes([m]) + struct.pack('<h', int(round(rpm))))

    def set_ctrl(self, m, cfg):
        # 字段顺序必须与 comm_task.c 的解析一致: kp,ki,ff,ff_offset,stiction,slew,fade
        data = bytes([m]) + struct.pack('<fffhhhh',
                                        cfg['kp'], cfg['ki'], cfg['ff'],
                                        int(cfg['ff_offset']), int(cfg['stiction']),
                                        int(cfg['slew']), int(cfg['fade']))
        self._send(SET_CTRL, data)

    def telemetry(self, timeout=0.12):
        self._send(GET_TEL)
        f = self._wait(lambda fr: fr[3] == TEL and fr[1] == 24, timeout)
        if f is None:
            return None
        return list(struct.unpack('<4h', f[4:12])), list(struct.unpack('<4i', f[12:28]))

    def battery(self):
        """读一次电池: (电压V, 电量%, 剩余mAh) 或 None; 用于记录标定时的电压"""
        self._send(GET_BAT)
        f = self._wait(lambda fr: fr[3] == BAT and fr[1] == 6, 0.3)
        if f is None:
            return None
        return (struct.unpack('<H', f[4:6])[0] / 1000.0, f[6],
                struct.unpack('<H', f[7:9])[0])

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SimMotor:
    """与固件 motor_service_poll 同构的被控对象: 一阶+死区+M/T量化+测速低通+增量式PI"""

    def __init__(self, K, u0, tau, seed=0):
        self.K, self.u0, self.tau = K, u0, tau
        self.v = 0.0                 # 真实转速
        self.pos = 0.0               # 真实累计计数
        self.mode = 'open'
        self.cmd_duty = 0.0
        self.braking = False
        self.target = 0
        self.ref = 0.0
        self.pid = 0.0
        self.prev_err = 0.0
        self.meas = 0.0              # 固件测速结果(带低通)
        self.cnt = 0.0
        self.win_us = 0
        self.cfg = {'kp': 1.0, 'ki': 1.0, 'ff': 1.0, 'ff_offset': 0,
                    'stiction': 0, 'slew': 0, 'fade': 0}
        self.rng = random.Random(seed)
        self.freeze_int = False      # what-if: 斜坡期间冻结积分(对应固件补丁)

    def brake(self):
        self.mode, self.target, self.cmd_duty = 'open', 0, 0.0
        self.braking = True

    def set_duty(self, u):
        self.mode, self.target, self.braking = 'open', 0, False
        self.cmd_duty = clamp(float(u), -DUTY_MAX, DUTY_MAX)

    def set_speed(self, rpm):
        if self.mode != 'speed':                 # 固件: 开环->闭环时 reset_control
            self.pid = self.prev_err = self.ref = 0.0
            self.cnt = 0.0
            self.win_us = 0
        self.mode, self.target, self.braking = 'speed', int(rpm), False

    def set_ctrl(self, cfg):
        self.cfg = dict(cfg)

    def tick(self, dt=CTRL_DT):
        # --- M/T 自适应窗口测速(判据与固件相同: |M|>=8 或 T>=50ms) ---
        self.cnt += self.v * ENC_CPR / 60.0 * dt + self.rng.gauss(0.0, 0.30)
        self.win_us += int(dt * 1e6)
        if abs(self.cnt) >= 8.0 or self.win_us >= 50000:
            raw = self.cnt * 60e6 / (ENC_CPR * self.win_us)
            self.meas += LPF_ALPHA * (raw - self.meas)
            self.cnt, self.win_us = 0.0, 0

        # --- 速度环(与 motor_service_poll 逐行对应) ---
        if self.mode == 'speed':
            cfg = self.cfg
            step = cfg['slew'] * dt
            diff = self.target - self.ref
            if step <= 0.0:
                self.ref = float(self.target)
            elif diff > step:
                self.ref += step
            elif diff < -step:
                self.ref -= step
            else:
                self.ref = float(self.target)

            if self.target == 0 and abs(self.meas) < 5.0:
                out = 0.0
                self.prev_err = self.pid = self.ref = 0.0
            else:
                err = self.ref - self.meas
                du = cfg['kp'] * (err - self.prev_err)
                if not (self.freeze_int and abs(self.ref - self.target) > 0.01):
                    du += cfg['ki'] * err * dt
                self.prev_err = err
                self.pid = clamp(self.pid + du, -DUTY_MAX, DUTY_MAX)

                ff = cfg['ff'] * self.ref + cfg['ff_offset'] * sgn(self.target)
                kick = 0.0
                if self.target != 0:
                    spd = abs(self.meas)
                    w = (1.0 - spd / cfg['fade']) if cfg['fade'] > 0 else 0.0
                    kick = cfg['stiction'] * max(0.0, w) * sgn(self.target)
                out = ff + kick + self.pid
            self.cmd_duty = clamp(out, -DUTY_MAX, DUTY_MAX)

        # --- 被控对象 ---
        if self.braking:
            self.v *= math.exp(-dt / 0.05)
        else:
            u = self.cmd_duty
            vss = 0.0
            if abs(u) > self.u0:
                vss = self.K * (u - math.copysign(self.u0, u))
            self.v += (vss - self.v) * min(1.0, dt / self.tau)
        self.pos += self.v * ENC_CPR / 60.0 * dt


class SimChannel:
    """虚拟通道: 每次遥测推进一个控制周期, 时间与真机同为 10ms 网格"""

    def __init__(self, specs, freeze=False):
        self.t = 0.0
        self.motors = [SimMotor(s['K'], s['u0'], s['tau'], seed=100 + i)
                       for i, s in enumerate(specs)]
        for mo in self.motors:
            mo.freeze_int = freeze
        self.info = {'port': 'SIM'}
        self.rtt_ms = 10.0

    def now(self):
        return self.t

    def sleep(self, s):
        for _ in range(max(0, int(round(s / CTRL_DT)))):
            self._tick()

    def _tick(self):
        for mo in self.motors:
            mo.tick()
        self.t += CTRL_DT

    def ping(self):
        return True

    def brake(self, m):
        self.motors[m].brake()

    def set_duty(self, m, duty):
        self.motors[m].set_duty(duty)

    def set_speed(self, m, rpm):
        self.motors[m].set_speed(rpm)

    def set_ctrl(self, m, cfg):
        self.motors[m].set_ctrl(cfg)

    def telemetry(self, timeout=0.12):
        self._tick()
        return ([int(mo.meas) for mo in self.motors],
                [int(mo.pos) for mo in self.motors])

    def battery(self):
        return (12.4, 90, 2000)

    def close(self):
        pass


# ================================================================ 采样子程序
def wait_still(ch, m, timeout=2.5):
    """刹车并等到连续 3 帧 |rpm|<=2"""
    ch.brake(m)
    ch.sleep(0.35)
    ok, t0 = 0, ch.now()
    while ch.now() - t0 < timeout:
        tel = ch.telemetry()
        if tel is None:
            continue
        if abs(tel[0][m]) <= 2:
            ok += 1
            if ok >= 3:
                return True
        else:
            ok = 0
    return False


def _pos_speed(samples, t_end, win):
    """位置差分测速: (pos_b-pos_a)*60/(CPR*dt), 精度远高于 int16 rpm"""
    b = None
    for s in samples:
        if s[0] <= t_end:
            b = s
    if b is None:
        return None
    a = None
    for s in samples:
        if s[0] >= b[0] - win:
            a = s
            break
    if a is None or b[0] - a[0] < 0.1:
        return None
    return (b[2] - a[2]) * 60.0 / (ENC_CPR * (b[0] - a[0]))


def _settled(samples, win=0.25, tol=1.2):
    """连续两个 0.25s 窗口的位置速度差 < tol rpm -> 认为已稳态"""
    if len(samples) < 20:
        return False
    t_end = samples[-1][0]
    if t_end < 2 * win + 0.05:
        return False
    v2 = _pos_speed(samples, t_end, win)
    v1 = _pos_speed(samples, t_end - win, win)
    return v1 is not None and v2 is not None and abs(v2 - v1) < tol


def record(ch, m, seconds):
    t0 = ch.now()
    out = []
    while ch.now() - t0 < seconds:
        tel = ch.telemetry()
        if tel is None:
            continue
        out.append((ch.now() - t0, tel[0][m], tel[1][m]))
    return out


def step_capture(ch, m, duty, max_dur=3.5, min_dur=0.7):
    """开环阶跃: 从静止给占空比, 采到稳态或超时"""
    ch.set_duty(m, duty)
    samples, t0 = [], ch.now()
    while True:
        tel = ch.telemetry()
        if tel is None:
            continue
        t = ch.now() - t0
        samples.append((t, tel[0][m], tel[1][m]))
        if t >= min_dur and _settled(samples):
            break
        if t >= max_dur:
            break
    ch.set_duty(m, 0)
    return samples


# ================================================================ 辨识
TAU_GRID = [0.02 * (1.06 ** i) for i in range(72)]       # 0.02 ~ 1.19 s
L_GRID = [0.01 * i for i in range(9)]                    # 0 ~ 80 ms


def fit_tau(samples, A):
    """在 (L, tau) 网格上最小化 SSE:
       真实转速 A*(1-exp(-(t-L)/tau)), 再过一个 alpha=0.35@100Hz 的测速低通
       注: 高占空比时电机噪声会打断串口, 丢掉间隔过大的点避免污染时间轴"""
    ts, ys, prev = [], [], None
    for t, rpm, _ in samples:
        if prev is None or (t - prev) <= 0.03:
            ts.append(t)
            ys.append(float(rpm))
        prev = t
    if len(ys) < 20:
        raise RuntimeError('阶跃样本不足(%d)' % len(ys))
    best = None
    for tau in TAU_GRID:
        for L in L_GRID:
            y, sse, prev = 0.0, 0.0, ts[0]
            for t, yr in zip(ts, ys):
                dt = max(1e-3, t - prev)
                prev = t
                v = 0.0 if t <= L else A * (1.0 - math.exp(-(t - L) / tau))
                a = 1.0 - (1.0 - LPF_ALPHA) ** (dt / CTRL_DT)
                y += a * (v - y)
                d = y - yr
                sse += d * d
            if best is None or sse < best[0]:
                best = (sse, tau, L)
    return best[1], best[2], math.sqrt(best[0] / len(ys))


def fit_static(points):
    """最小二乘 v = K*u + b -> K, u0=-b/K, R²"""
    pts = [(u, v) for u, v in points if v is not None and v > 8.0]
    if len(pts) < 3:
        raise RuntimeError('有效工作点不足(%d), 检查电机是否转动/占空比是否低于死区' % len(pts))
    n = len(pts)
    su = sum(u for u, _ in pts)
    sv = sum(v for _, v in pts)
    suu = sum(u * u for u, _ in pts)
    suv = sum(u * v for u, v in pts)
    den = n * suu - su * su
    if abs(den) < 1e-9:
        raise RuntimeError('工作点占空比无区分度')
    k = (n * suv - su * sv) / den
    b = (sv - k * su) / n
    if k <= 1e-6:
        raise RuntimeError('拟合斜率 <= 0, 辨识失败')
    u0 = -b / k
    ss_tot = sum((v - sv / n) ** 2 for _, v in pts)
    ss_res = sum((v - (k * u + b)) ** 2 for u, v in pts)
    return k, u0, (1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0), pts


def interp_tau(points, target):
    pts = sorted((p['v'], p['tau']) for p in points if p['v'] and p['tau'])
    if not pts:
        return 0.2
    if target <= pts[0][0]:
        return pts[0][1]
    if target >= pts[-1][0]:
        return pts[-1][1]
    for (v1, t1), (v2, t2) in zip(pts, pts[1:]):
        if v1 <= target <= v2:
            w = (target - v1) / (v2 - v1) if v2 > v1 else 0.0
            return t1 + w * (t2 - t1)
    return pts[-1][1]


# ================================================================ 设计
def design(k, u0, tau, ratio, slew, fade, stic_scale):
    """极点对消 PI + 前馈 + 静摩擦补偿"""
    tau_cl = max(0.02, tau / ratio)
    return {
        'kp': tau / (k * tau_cl),
        'ki': 1.0 / (k * tau_cl),
        'ff': 1.0 / k,
        'ff_offset': int(round(u0)),
        'stiction': int(clamp(round(u0 * 0.2 * stic_scale), 0, 250)),
        'slew': int(slew),
        'fade': int(fade),
    }


# ================================================================ 验收
def metrics(samples, target):
    t = [s[0] for s in samples]
    y = [float(s[1]) for s in samples]
    end = t[-1]

    steady = _pos_speed(samples, end, 0.8)
    if steady is None:
        steady = statistics.mean(y[-10:])
    sse = steady - target

    tail = [v for tt, v in zip(t, y) if tt >= end - 0.8]
    # 纹波用"掐头去尾 1%"的稳健标准差: 单个丢计数毛刺不该毁掉整份结论,
    # 毛刺另用 spikes 计数单独报告
    if len(tail) > 4:
        srt = sorted(tail)
        k = max(0, int(len(srt) * 0.01))
        core = srt[k:len(srt) - k] if len(srt) > 2 * k else srt
        ripple = statistics.pstdev(core)
    else:
        ripple = 0.0
    spikes = sum(1 for v in tail if abs(v - target) > max(5.0, 0.05 * abs(target)))

    # 5 点中值滤波: 抑制 int16 量化抖动, 避免把噪声当成过冲/未收敛
    sm = [statistics.median(y[max(0, i - 2):i + 3]) for i in range(len(y))]
    peak = max(sm, key=abs)
    over = (abs(peak) - abs(target)) / abs(target) * 100.0 if target else 0.0

    band = max(1.0, 0.02 * abs(target))
    settle = None
    for i in range(len(sm)):
        if all(abs(v - target) <= band for v in sm[i:]):
            settle = t[i]
            break

    return {'target': target, 'steady': steady, 'sse': sse, 'ripple': ripple,
            'peak': peak, 'overshoot': over, 'settle': settle, 'spikes': spikes}


def _spike_note(r):
    """少量毛刺单独标注, 不计入纹波"""
    return ('  毛刺%d' % r['spikes']) if r.get('spikes') else ''


def verify(ch, m, targets, hold=2.6):
    """返回 (指标行, 原始曲线); 曲线留给报告图用"""
    rows, traces = [], []
    for tgt in targets:
        wait_still(ch, m)
        ch.set_speed(m, int(tgt))
        s = record(ch, m, hold)
        rows.append(metrics(s, tgt))
        traces.append((int(tgt), s))
        ch.set_speed(m, 0)
    return rows, traces


# ================================================================ 纯验收
def verify_motor(ch, m, args, log):
    """不重新辨识、不下发 SET_CTRL: 只验证固件当前缺省参数的表现"""
    name = 'M%d' % (m + 1)
    bat = ch.battery()
    targets = [int(x) for x in args.targets] if args.targets else [75, 150, 240]
    log('')
    log('======== %s 验收（固件当前参数，不下发 SET_CTRL） ========' % name)
    log('  电池 %s' % ('%.2f V / %d%% / %d mAh' % bat if bat else '读取失败'))
    log('  目标: %s rpm' % targets)

    wait_still(ch, m)
    rows, traces = verify(ch, m, targets, args.hold)
    for r in rows:
        log('     %5d rpm  SSE %+6.2f  纹波 %.2f  过冲 %+5.1f%%  调节 %s%s'
            % (r['target'], r['sse'], r['ripple'], r['overshoot'],
               ('%.2fs' % r['settle']) if r['settle'] is not None else '-', _spike_note(r)))

    fails = []
    if any(abs(r['overshoot']) > args.max_overshoot for r in rows):
        fails.append('过冲>%.0f%%' % args.max_overshoot)
    if any(abs(r['sse']) > args.max_sse for r in rows):
        fails.append('稳态误差>%.1frpm' % args.max_sse)
    if any(r['ripple'] > args.max_ripple for r in rows):
        fails.append('纹波>%.1frpm' % args.max_ripple)

    ch.set_speed(m, 0)
    wait_still(ch, m)
    return {'motor': m, 'name': name, 'ident': None, 'cfg': None, 'rows': rows,
            'reverse': None, 'traces': traces, 'ok': not fails, 'fails': fails,
            'ratio': None, 'attempt': 0, 'battery': bat}


# ================================================================ 单个电机
def tune_motor(ch, m, args, log):
    name = 'M%d' % (m + 1)
    log('')
    log('======== %s 开环辨识 ========' % name)
    bat = ch.battery()
    log('  电池 %s (标定时点, u0 会随电压/温度漂移)'
        % ('%.2f V / %d%% / %d mAh' % bat if bat else '读取失败'))

    levels = [int(round(args.duty_low + (args.duty_high - args.duty_low) * i / (args.levels - 1)))
              for i in range(args.levels)]
    points = []
    for duty in levels:
        if not wait_still(ch, m):
            log('  duty %4d  ⚠ 未停稳, 跳过' % duty)
            continue
        s = step_capture(ch, m, duty, args.max_step)
        a = _pos_speed(s, s[-1][0], 0.6)
        tau = L = rmse = None
        if a is not None and abs(a) > 5.0:
            tau, L, rmse = fit_tau(s, a)
        points.append({'duty': duty, 'v': a, 'tau': tau, 'L': L, 'rmse': rmse, 'n': len(s)})
        log('  duty %4d  steady %7.1f rpm  tau %-6s  L %-6s  rmse %-5s  (%d pts)'
            % (duty, a if a is not None else float('nan'),
               ('%.3f' % tau) if tau else '-', ('%.2f' % L) if L is not None else '-',
               ('%.1f' % rmse) if rmse else '-', len(s)))

    vals = [p['v'] for p in points if p['v'] is not None and abs(p['v']) > 5.0]
    if vals and all(v < 0 for v in vals):
        raise RuntimeError('正占空比得到负转速: 电机/编码器方向不一致, 先检查接线')

    k, u0, r2, pts = fit_static([(p['duty'], p['v']) for p in points])
    log('  K = %.4f rpm/‰   u0 = %.1f ‰   R² = %.5f' % (k, u0, r2))
    if r2 < 0.995:
        log('  ⚠ 线性度偏低(R²=%.4f), 工作点可能在死区附近或存在饱和' % r2)

    vmax = max(v for _, v in pts)
    if args.targets:
        targets = [int(x) for x in args.targets]
    else:
        targets = sorted({max(30, int(round(vmax * f))) for f in (0.25, 0.5, 0.8)})
    log('  验收目标: %s rpm   (vmax≈%.0f)' % (targets, vmax))

    best = None
    ratio, stic, ki_scale = args.ratio, 1.0, 1.0
    for att in range(1, args.attempts + 1):
        tau_t = interp_tau(points, statistics.mean(targets))
        cfg = design(k, u0, tau_t, ratio, args.slew, args.fade, stic)
        cfg['ki'] *= ki_scale
        ch.set_ctrl(m, cfg)
        rows, traces = verify(ch, m, targets, args.hold)
        over_v = sum(max(0.0, abs(r['overshoot']) - args.max_overshoot) for r in rows)
        sse_v = sum(max(0.0, abs(r['sse']) - args.max_sse) for r in rows)
        rip_v = sum(max(0.0, r['ripple'] - args.max_ripple) for r in rows)
        score = (over_v / max(1e-6, args.max_overshoot) + sse_v / args.max_sse
                 + rip_v / args.max_ripple)
        log('  -- 第 %d 轮  ratio=%.2f tau_cl=%.3f  kp=%.3f ki=%.3f(×%.2f) stiction=%d  违规分=%.3f'
            % (att, ratio, max(0.02, tau_t / ratio), cfg['kp'], cfg['ki'], ki_scale,
               cfg['stiction'], score))
        for r in rows:
            log('     %5d rpm  SSE %+6.2f  纹波 %.2f  过冲 %+5.1f%%  调节 %s%s'
                % (r['target'], r['sse'], r['ripple'], r['overshoot'],
                   ('%.2fs' % r['settle']) if r['settle'] is not None else '  >%.1fs' % args.hold,
                   _spike_note(r)))
        if best is None or score < best['score']:
            best = {'score': score, 'cfg': cfg, 'rows': rows, 'traces': traces, 'ratio': ratio,
                    'tau_t': tau_t, 'attempt': att, 'ki_scale': ki_scale}
        if score <= 1e-9:
            break
        # 过冲主要来自斜坡期间的积分累积 -> 降 ki; 稳态不足 -> 升 ki
        if over_v > 0:
            ki_scale = max(0.15, ki_scale * 0.6)
            stic *= 0.7
        if sse_v > 0:
            ki_scale = min(2.0, ki_scale * 1.4)

    # 恢复最优参数
    ch.set_ctrl(m, best['cfg'])

    rev = None
    if not args.no_reverse:
        rev_rows, _ = verify(ch, m, [-abs(targets[len(targets) // 2])], hold=args.hold)
        rev = rev_rows[0]
        log('  -- 反向校验 %d rpm: SSE %+.2f  过冲 %+.1f%%'
            % (rev['target'], rev['sse'], rev['overshoot']))

    ok = best['score'] <= 1e-9
    fails = []
    if any(abs(r['overshoot']) > args.max_overshoot for r in best['rows']):
        fails.append('过冲>%.0f%%' % args.max_overshoot)
    if any(abs(r['sse']) > args.max_sse for r in best['rows']):
        fails.append('稳态误差>%.1frpm' % args.max_sse)
    if any(r['ripple'] > args.max_ripple for r in best['rows']):
        fails.append('纹波>%.1frpm' % args.max_ripple)
    ch.set_speed(m, 0)
    wait_still(ch, m)

    return {
        'motor': m, 'name': name,
        'ident': {'K': k, 'u0': u0, 'r2': r2,
                  'tau': best['tau_t'], 'points': points},
        'cfg': best['cfg'], 'rows': best['rows'], 'reverse': rev,
        'ratio': best['ratio'], 'attempt': best['attempt'], 'ok': ok,
        'fails': fails, 'battery': bat, 'traces': best['traces'],
    }


# ================================================================ 报告
def plot_report(results, path, show=False):
    """标定报告图: 每台电机 4 联图 —— 静态特性 / 阶跃响应 / 稳态放大 / 指标柱"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print('未安装 matplotlib, 跳过作图: python -m pip install matplotlib')
        return None

    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    n = max(1, len(results))
    fig, axes = plt.subplots(4, n, figsize=(6.4 * n, 13.5), squeeze=False)

    for c, r in enumerate(results):
        ident = r['ident']

        # (0) 开环静态特性: 证明辨识可信
        ax = axes[0][c]
        if ident is None:
            ax.set_axis_off()
            ax.text(0.5, 0.55, '固件当前参数\n（未重新辨识）', ha='center', va='center',
                    transform=ax.transAxes, fontsize=13, color='0.35')
            if r.get('battery'):
                ax.text(0.5, 0.25, '电池 %.2f V / %d%%' % (r['battery'][0], r['battery'][1]),
                        ha='center', va='center', transform=ax.transAxes, fontsize=10, color='0.45')
        else:
            pts = [p for p in ident['points'] if p['v'] is not None and abs(p['v']) > 5]
            if pts:
                d = [p['duty'] for p in pts]
                v = [p['v'] for p in pts]
                ax.plot(d, v, 'o', ms=6, color='tab:blue', label='实测稳态点')
                uu = np.linspace(min(d), max(d), 60)
                ax.plot(uu, ident['K'] * (uu - ident['u0']), '-', color='tab:orange',
                        lw=1.3, label='拟合 v = K(u − u0)')
            ax.set_title('%s 开环静态特性' % r['name'], fontsize=11)
            ax.set_xlabel('占空比 (‰)')
            ax.set_ylabel('稳态转速 (rpm)')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8, loc='lower right')
            ax.text(0.03, 0.97,
                    'K = %.4f rpm/‰\nu0 = %.1f ‰\nR² = %.4f' % (ident['K'], ident['u0'], ident['r2']),
                    transform=ax.transAxes, va='top', fontsize=9,
                    bbox=dict(fc='white', ec='0.85', alpha=0.95))

        # (1) 阶跃响应: 看动态
        ax = axes[1][c]
        for row, (tgt, s) in zip(r['rows'], r['traces']):
            lab = '%d rpm｜过冲 %+.1f%%｜调节 %s' % (
                tgt, row['overshoot'],
                ('%.2f s' % row['settle']) if row['settle'] is not None else '-')
            ax.plot([x[0] for x in s], [x[1] for x in s], lw=1.2, label=lab)
            ax.axhline(tgt, color='0.55', ls='--', lw=0.8)
        ax.set_title('%s 阶跃响应' % r['name'], fontsize=11)
        ax.set_xlabel('t (s)')
        ax.set_ylabel('转速 (rpm)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc='lower right')

        # (2) 稳态误差残差: 三条线叠在一起才看得出精度
        ax = axes[2][c]
        for row, (tgt, s) in zip(r['rows'], r['traces']):
            t_end = s[-1][0]
            seg = [(x[0] - t_end, x[1] - tgt) for x in s if x[0] >= t_end - 1.2]
            ax.plot([p[0] for p in seg], [p[1] for p in seg], lw=1.1,
                    label='%d rpm｜SSE %+.2f｜纹波 %.2f%s'
                          % (tgt, row['sse'], row['ripple'],
                             ('｜毛刺%d' % row['spikes']) if row.get('spikes') else ''))
        ax.axhline(0.0, color='0.55', ls='--', lw=0.8)
        ax.axhspan(-1.0, 1.0, color='tab:green', alpha=0.12, lw=0)
        ax.set_title('%s 稳态误差（最后 1.2 s，绿带 ±1 rpm）' % r['name'], fontsize=11)
        ax.set_xlabel('t − t_end (s)')
        ax.set_ylabel('实测 − 目标 (rpm)')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc='best')

        # (3) 指标柱: 左轴稳态误差(rpm), 右轴过冲(%), 一眼看是否达标
        ax = axes[3][c]
        idx = np.arange(len(r['rows']))
        ax.bar(idx - 0.2, [abs(x['sse']) for x in r['rows']], width=0.38,
               color='tab:blue', label='|稳态误差| (rpm)')
        ax.axhline(1.0, color='tab:green', ls=':', lw=1.0)
        ax.set_ylabel('|稳态误差| (rpm)', color='tab:blue')
        ax.tick_params(axis='y', labelcolor='tab:blue')

        ax2 = ax.twinx()
        ax2.bar(idx + 0.2, [x['overshoot'] for x in r['rows']], width=0.38,
                color='tab:orange', label='过冲 (%)')
        ax2.axhline(5.0, color='tab:red', ls=':', lw=1.0)
        ax2.set_ylabel('过冲 (%)', color='tab:orange')
        ax2.tick_params(axis='y', labelcolor='tab:orange')

        ax.set_xticks(idx)
        ax.set_xticklabels(['%d rpm' % x['target'] for x in r['rows']])
        ax.set_title('%s 指标（绿线 1 rpm / 红线 5%%）' % r['name'], fontsize=11)
        ax.grid(alpha=0.3, axis='y')
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc='upper left')

    bat = results[0].get('battery') if results else None
    sub = 'XBot core_control 速度环标定    %s' % time.strftime('%Y-%m-%d %H:%M')
    if bat:
        sub += '    电池 %.2f V / %d%%' % (bat[0], bat[1])
    fig.suptitle(sub, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.975])
    fig.savefig(path, dpi=140)
    if show:
        plt.show()
    plt.close(fig)
    return path


def run_paths(args, mode='tune'):
    """一次运行的两个产物路径(同一时间戳, 便于配对):
       标定: result/tune_report_<stamp>.png   + tune_data/tune_results_<stamp>.json
       验收: result/verify_report_<stamp>.png + tune_data/verify_results_<stamp>.json
       显式给了 --plot-path / --json 时用指定的路径。"""
    stamp = time.strftime('%Y%m%d_%H%M%S')
    base = os.path.dirname(os.path.abspath(__file__))
    prefix = 'verify' if mode == 'verify' else 'tune'
    img_dir = os.path.join(base, 'result')
    json_dir = args.json_dir or os.path.join(base, 'tune_data')
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(json_dir, exist_ok=True)
    img = args.plot_path or os.path.join(img_dir, '%s_report_%s.png' % (prefix, stamp))
    js = args.json or os.path.join(json_dir, '%s_results_%s.json' % (prefix, stamp))
    return img, js


def print_report(results, args):
    print('\n================ 汇总 ================')
    for r in results:
        status = 'OK' if r['ok'] else ('未达标: ' + ', '.join(r['fails']) if r.get('fails') else '未达标')
        if r['ident']:
            print('%s  K=%.4f  u0=%.1f  tau=%.3f  %s'
                  % (r['name'], r['ident']['K'], r['ident']['u0'], r['ident']['tau'], status))
        else:
            print('%s  固件当前参数  %s' % (r['name'], status))
        for row in r['rows']:
            print('   %5d rpm  SSE %+6.2f  ripple %.2f  over %+5.1f%%  settle %s%s'
                  % (row['target'], row['sse'], row['ripple'], row['overshoot'],
                     ('%.2fs' % row['settle']) if row['settle'] is not None else '-',
                     _spike_note(row)))
        if r['reverse']:
            print('   %5d rpm  SSE %+6.2f  over %+5.1f%%  (reverse)'
                  % (r['reverse']['target'], r['reverse']['sse'], r['reverse']['overshoot']))
        c = r['cfg']
        if c:
            print('   cfg: kp=%.4f ki=%.4f ff=%.4f ff_offset=%d stiction=%d slew=%d fade=%d'
                  % (c['kp'], c['ki'], c['ff'], c['ff_offset'], c['stiction'], c['slew'], c['fade']))


def c_snippet(results, n_motor=4):
    lines = ['/* autotune2 标定结果 - 注意结构体字段顺序:',
             ' * kp, ki, ff_gain, ff_offset, slew_rpm_s, fade_rpm, stiction',
             ' * (与 SET_CTRL 载荷顺序不同, 别抄错) */',
             'static const motor_ctrl_cfg_t k_motor_default[MOTOR_NUM] = {']
    by_id = {r['motor']: r for r in results if r.get('cfg')}
    for i in range(n_motor):
        if i in by_id:
            c = by_id[i]['cfg']
            lines.append('    [%d] = { %.4ff, %.4ff, %.4ff, %d, %d, %d, %d },  /* %s */'
                         % (i, c['kp'], c['ki'], c['ff'], c['ff_offset'],
                            c['slew'], c['fade'], c['stiction'], by_id[i]['name']))
        else:
            lines.append('    [%d] = { MOTOR_DEFAULT_KP, MOTOR_DEFAULT_KI, MOTOR_DEFAULT_FF_GAIN,'
                         ' MOTOR_DEFAULT_FF_OFFSET, MOTOR_DEFAULT_SLEW_RPM_S,'
                         ' MOTOR_DEFAULT_FADE_RPM, MOTOR_DEFAULT_STICTION },' % i)
    lines.append('};')
    return '\n'.join(lines)


def save_json(path, results, args, mode='tune'):
    """只保留关键数据: 辨识摘要 + 固件参数(cfg) + 验收指标。
       辨识原始点/曲线/完整命令行参数都不入库, 需要回溯看报告图和终端输出。"""
    def metric(r):
        d = {'target': r['target'],
             'sse': round(r['sse'], 3),
             'ripple': round(r['ripple'], 3),
             'overshoot': round(r['overshoot'], 2),
             'settle': round(r['settle'], 3) if r['settle'] is not None else None}
        if r.get('spikes'):
            d['spikes'] = r['spikes']
        return d

    doc = {'tool': 'autotune2', 'mode': mode,
           'time': time.strftime('%Y-%m-%d %H:%M:%S'),
           'port': getattr(args, 'port', None),
           'motors': []}

    bat = next((r['battery'] for r in results if r.get('battery')), None)
    if bat:
        doc['battery'] = {'voltage_v': round(bat[0], 2), 'soc_pct': bat[1]}

    for r in results:
        ident = r['ident'] or {}
        cfg = r.get('cfg')
        item = {
            'id': r['motor'], 'name': r['name'],
            'K': round(ident['K'], 4) if ident.get('K') is not None else None,
            'u0': round(ident['u0'], 1) if ident.get('u0') is not None else None,
            'r2': round(ident['r2'], 4) if ident.get('r2') is not None else None,
            'tau': round(ident['tau'], 3) if ident.get('tau') is not None else None,
            # 固件参数: 直接对应 motor_ctrl_cfg_t, 可直接抄进 k_motor_default[]
            'cfg': ({k: (round(v, 4) if isinstance(v, float) else v) for k, v in cfg.items()}
                    if cfg else None),
            'verify': [metric(x) for x in r['rows']],
            'ok': r['ok'],
        }
        if r.get('fails'):
            item['fails'] = r['fails']
        if r.get('reverse'):
            item['reverse'] = metric(r['reverse'])
        doc['motors'].append(item)

    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)
    return path


# ================================================================ 离线自测
def dry_run(args):
    print('== DRY RUN: 合成被控对象, 不接硬件 ==')
    truth = [{'K': 0.62, 'u0': 430.0, 'tau': 0.24},
             {'K': 0.57, 'u0': 452.0, 'tau': 0.27},
             {'K': 0.60, 'u0': 440.0, 'tau': 0.25},
             {'K': 0.60, 'u0': 440.0, 'tau': 0.25}]
    ch = SimChannel(truth, freeze=args.sim_freeze)
    print('  sim freeze_int = %s' % args.sim_freeze)
    args.no_reverse = True
    results = [tune_motor(ch, m, args, print) for m in (0, 1)]
    print_report(results, args)

    print('\n== 辨识精度(真值 -> 反解) ==')
    okall = True
    for r in results:
        t = truth[r['motor']]
        ek = (r['ident']['K'] - t['K']) / t['K'] * 100
        e0 = r['ident']['u0'] - t['u0']
        et = (r['ident']['tau'] - t['tau']) / t['tau'] * 100
        good = abs(ek) < 3.0 and abs(e0) < 8.0 and abs(et) < 15.0
        okall &= good
        print('%s  K %.3f->%.3f (%+.1f%%)  u0 %.0f->%.0f (%+.1f)  tau %.3f->%.3f (%+.1f%%)  %s'
              % (r['name'], t['K'], r['ident']['K'], ek, t['u0'], r['ident']['u0'], e0,
                 t['tau'], r['ident']['tau'], et, 'PASS' if good else 'FAIL'))

    print('\n== 闭环指标(仿真) ==')
    for r in results:
        for row in r['rows']:
            print('%s %5d rpm  SSE %+6.2f  ripple %.2f  over %+5.1f%%  settle %s'
                  % (r['name'], row['target'], row['sse'], row['ripple'], row['overshoot'],
                     ('%.2fs' % row['settle']) if row['settle'] is not None else '-'))
    print('\n%s' % ('DRY RUN PASS' if okall else 'DRY RUN FAIL'))
    return 0 if okall else 1


# ================================================================ main
def main():
    ap = argparse.ArgumentParser(description='逐电机精密整定 (速度环)')
    ap.add_argument('port', nargs='?', default='COM7')
    ap.add_argument('--motor', type=int, nargs='+', default=[0, 1], help='要标定的电机 id, 默认 0 1')
    ap.add_argument('--levels', type=int, default=6, help='开环阶跃档数')
    ap.add_argument('--duty-low', type=int, default=480)
    ap.add_argument('--duty-high', type=int, default=900)
    ap.add_argument('--max-step', type=float, default=3.5, help='单次阶跃最长采样(s)')
    ap.add_argument('--ratio', type=float, default=1.5, help='闭环带宽相对开环的倍数')
    ap.add_argument('--slew', type=int, default=250,
                    help='参考爬升率 rpm/s (0=关斜坡; 默认 250, 与固件 MOTOR_DEFAULT_SLEW_RPM_S 一致)')
    ap.add_argument('--fade', type=int, default=50, help='静摩擦补偿衰减区间 rpm')
    ap.add_argument('--attempts', type=int, default=3, help='自动重试轮数')
    ap.add_argument('--hold', type=float, default=2.6, help='闭环验收单次采样(s)')
    ap.add_argument('--targets', type=int, nargs='+', default=None, help='指定验收转速')
    ap.add_argument('--max-overshoot', type=float, default=5.0, help='过冲门限 %')
    ap.add_argument('--max-sse', type=float, default=1.5, help='稳态误差门限 rpm')
    ap.add_argument('--max-ripple', type=float, default=1.5, help='纹波门限 rpm')
    ap.add_argument('--verify-only', action='store_true',
                    help='只验收固件当前参数(不重新辨识、不下发 SET_CTRL), 用于认证已烧录的缺省值')
    ap.add_argument('--no-reverse', action='store_true')
    ap.add_argument('--no-plot', action='store_true', help='不生成标定报告图')
    ap.add_argument('--show', action='store_true', help='画完弹窗显示')
    ap.add_argument('--plot-path', default=None, help='报告图输出路径')
    ap.add_argument('--json', default=None,
                    help='结果 JSON 全路径(默认 tools/tune_data/tune_results_<时间戳>.json)')
    ap.add_argument('--json-dir', default=None, help='结果 JSON 目录(默认 tools/tune_data)')
    ap.add_argument('--dry-run', action='store_true', help='离线自测, 不动硬件')
    ap.add_argument('--check', action='store_true', help='只读自检: PING + 遥测, 不驱动电机')
    ap.add_argument('--sim-freeze', action='store_true',
                    help='配合 --dry-run: 仿真"斜坡期间冻结积分"的固件补丁, 用来看补丁收益')
    args = ap.parse_args()

    if args.dry_run:
        sys.exit(dry_run(args))

    ch = HostChannel(args.port)
    print('port %s  RTT %.1f ms' % (args.port, ch.rtt_ms or -1))
    if not ch.ping():
        print('PING 无应答, 检查端口/固件')
        ch.close()
        sys.exit(1)

    if args.check:
        print('只读自检: 连续 5 帧遥测(不驱动电机)')
        for _ in range(5):
            tel = ch.telemetry()
            print('  rpm=%s  pos=%s' % (tel[0] if tel else '-', tel[1] if tel else '-'))
        ch.close()
        return

    results = []
    try:
        # 标定前确认所有电机静止
        for m in range(4):
            ch.brake(m)
        ch.sleep(0.5)
        tel = ch.telemetry()
        if tel and any(abs(v) > 5 for v in tel[0]):
            print('⚠ 有电机仍在转: %s, 先停下再标定' % tel[0])
            sys.exit(1)

        for m in args.motor:
            if not (0 <= m < 4):
                print('电机 id 越界: %d' % m)
                continue
            try:
                if args.verify_only:
                    results.append(verify_motor(ch, m, args, print))
                else:
                    results.append(tune_motor(ch, m, args, print))
            except Exception as exc:
                print('\n[M%d] 标定失败: %s' % (m + 1, exc))
    except KeyboardInterrupt:
        print('\n中断, 刹车收尾')
    finally:
        for m in args.motor:
            try:
                ch.set_speed(m, 0)
                ch.brake(m)
            except Exception:
                pass
        ch.close()

    if results:
        print_report(results, args)
        print('\n---- C 片段 ----')
        print(c_snippet(results))
        mode = 'verify' if args.verify_only else 'tune'
        img_path, json_path = run_paths(args, mode)
        save_json(json_path, results, args, mode)
        print('\n结果已保存: %s' % json_path)
        if not args.no_plot:
            p = plot_report(results, img_path, args.show)
            if p:
                print('报告图已保存: %s' % p)
                if not args.plot_path:      # 固定名再存一份, 方便直接打开最新
                    latest = os.path.join(os.path.dirname(p), '%s_report.png' % mode)
                    shutil.copyfile(p, latest)
                    print('最新一份:     %s' % latest)


if __name__ == '__main__':
    main()
