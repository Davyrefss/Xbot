#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动调参脚本: 利用电机编码器测得的开环阶跃响应曲线, 拟合一阶模型并计算速度环参数。

原理(有刷直流+减速器近似一阶惯性):
    rpm(s) = K * (u - u0) / (tau*s + 1)
    K   = 增益 [rpm/千分比]
    u0  = 静摩擦死区 [千分比]
    tau = 时间常数 [s]

做法:
    1) 跑两次不同占空比的开环阶跃(u1, u2), 采转速曲线, 取稳态 A1, A2:
         K  = (A2 - A1) / (u2 - u1)     # 斜率
         u0 = u1 - A1 / K               # 死区截距
         tau = 由 63.2% 上升时间拟合
    2) 极点对消(PI 零点抵消电机极点, 闭环时间常数 tau_cl = tau / ratio):
         ff_gain   = 1 / K
         ff_offset = u0                  # 死区前馈偏移(稳态死区补偿)
         kp        = tau / (K * tau_cl)
         ki        = 1   / (K * tau_cl)
         stiction  = u0 * 0.2            # 静摩擦额外起步补偿(超出死区部分)
    3) 通过 SET_CTRL 广播下发, 再跑一次闭环阶跃验证。

用法:
    python autotune.py COM7
    python autotune.py COM7 --live --plot             # 实时滚动 + 结束静态汇总
    python autotune.py COM7 --csv curve.csv           # 存数据, 之后 plot_curve.py 回看
"""

import sys, os, time, struct, math, argparse
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_script import build, read_frame   # 复用帧协议工具

# 命令字
BRAKE, SET_SPEED, SET_DUTY, SET_CTRL, GET_TEL = 0x03, 0x01, 0x05, 0x06, 0x10
ACK, TEL = 0x80, 0x81

_seq = [0]


def next_seq():
    _seq[0] = (_seq[0] + 1) & 0xFF
    return _seq[0]


def send(ser, cmd, data=b''):
    ser.write(build(cmd, next_seq(), data))


def read_tel(ser, timeout=0.05):
    """读一帧 TELEMETRY, 返回 4 路 rpm 元组; 否则 None"""
    f = read_frame(ser, timeout)
    if f is not None and f[3] == TEL and f[1] == 24:
        return struct.unpack('<4h', f[4:12])
    return None


class LivePlot:
    """实时滚动曲线(交互模式): 采样过程中逐点重绘, 观察阶跃/闭环响应实时走势"""

    def __init__(self):
        import matplotlib.pyplot as plt
        plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        self.plt = plt
        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.ax.set_xlabel('t (s)')
        self.ax.set_ylabel('rpm')
        self.ax.grid(True)
        self.line = None
        self.xs = []
        self.ys = []
        plt.ion()
        self.fig.show()
        self.plt.pause(0.05)

    def begin(self, name, duration):
        """开始一个新阶段: 清空并重置坐标范围"""
        self.ax.cla()
        self.ax.set_title(name)
        self.ax.set_xlabel('t (s)')
        self.ax.set_ylabel('rpm')
        self.ax.grid(True)
        self.ax.set_xlim(0, duration)
        self.ax.set_ylim(0, 300)
        self.xs = []
        self.ys = []
        (self.line,) = self.ax.plot([], [], marker='.', markersize=2, color='tab:blue')
        self.fig.canvas.draw_idle()
        self.plt.pause(0.05)

    def add(self, t, y):
        """追加一个采样点并重绘, y 轴自适应"""
        self.xs.append(t)
        self.ys.append(y)
        self.line.set_data(self.xs, self.ys)
        if self.ys:
            lo, hi = min(self.ys), max(self.ys)
            pad = max(10.0, (hi - lo) * 0.15)
            self.ax.set_ylim(lo - pad, hi + pad)
        self.fig.canvas.draw_idle()
        self.plt.pause(0.01)

    def close(self):
        self.plt.close(self.fig)

    def finish(self):
        """结束: 关交互, 阻塞保留窗口直到手动关闭"""
        self.plt.ioff()
        self.plt.show()


def capture_step(ser, motor, duty, duration, live=None, live_name=None):
    """开环阶跃: 刹车->停稳->给 duty->采 rpm 曲线。返回 [(t, rpm), ...]"""
    send(ser, BRAKE, bytes([motor]))
    read_frame(ser, 0.15)                      # 吞掉 BRAKE 的 ACK
    time.sleep(0.4)                            # 等电机停稳

    samples = []
    if live:
        live.begin(live_name or '阶跃 %d' % duty, duration)
    t0 = time.time()
    send(ser, SET_DUTY, bytes([motor]) + struct.pack('<h', duty))
    read_frame(ser, 0.15)                      # 吞掉 SET_DUTY 的 ACK

    while time.time() - t0 < duration:
        send(ser, GET_TEL)
        rpm = read_tel(ser, 0.05)
        if rpm is not None:
            t = time.time() - t0
            samples.append((t, float(rpm[motor])))
            if live:
                live.add(t, float(rpm[motor]))

    send(ser, SET_DUTY, bytes([motor]) + struct.pack('<h', 0))   # 滑行停
    read_frame(ser, 0.15)
    return samples


def fit_first_order(samples):
    """拟合 rpm(t) = A*(1-exp(-(t-L)/tau))。返回 (A, tau, L)"""
    if len(samples) < 20:
        raise RuntimeError("样本太少(%d), 检查串口连接/编码器接线" % len(samples))

    n = len(samples)
    tail = sorted(r for _, r in samples[int(n * 0.7):])
    A = tail[len(tail) // 2]                   # 稳态转速: 尾部中位数
    if A < 5.0:
        raise RuntimeError("稳态转速≈%.1f rpm, 电机没转起来(占空比低于死区? 提高 --low/--high)" % A)

    L = 0.0
    for t, r in samples:
        if r >= 0.10 * A:
            L = t                              # 死区时间: 首次超过 10% 稳态
            break

    # ln(A-r) = ln(A) - (t-L)/tau  -> 线性回归求 tau
    xs, ys = [], []
    for t, r in samples:
        if t >= L and 0.02 * A < r < 0.95 * A:
            xs.append(t - L)
            ys.append(math.log(A - r))

    tau = None
    if len(xs) >= 6:
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        sxx = sum((x - mx) ** 2 for x in xs)
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        slope = sxy / sxx if sxx > 1e-12 else 0.0
        if slope < -1e-6:
            tau = -1.0 / slope

    if tau is None:                            # 退化: 63.2% 法
        for t, r in samples:
            if r >= 0.632 * A:
                tau = max(0.005, t - L)
                break
    if tau is None or tau <= 0:
        tau = 0.05
    return A, tau, L


def save_csv(path, steps):
    with open(path, 'w', encoding='utf-8') as f:
        f.write('step,t,rpm\n')
        for name, samples in steps:
            for t, r in samples:
                f.write('%s,%.4f,%.1f\n' % (name, t, r))


def plot_steps(steps, title=None):
    """用 matplotlib 画出各阶段 rpm-t 曲线。没装 matplotlib 时给出提示"""
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        print('  未安装 matplotlib: 先执行  python -m pip install matplotlib')
        print('  (或改用 --csv curve.csv 存数据, 用 Excel / plot_curve.py 画)')
        return False

    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    plt.figure(figsize=(10, 6))
    for name, samples in steps:
        if not samples:
            continue
        plt.plot([t for t, _ in samples], [r for _, r in samples],
                 marker='.', markersize=2, label=name)
    plt.xlabel('t (s)')
    plt.ylabel('rpm')
    plt.title(title or '电机阶跃/闭环响应')
    plt.legend()
    plt.grid(True)
    plt.ioff()          # 关闭交互模式, 保证 show() 阻塞等待手动关闭
    plt.show()
    return True


def main():
    ap = argparse.ArgumentParser(description='编码器阶跃响应自动整定')
    ap.add_argument('port')
    ap.add_argument('--motor', type=int, default=0, help='测试电机 id (0~3)')
    ap.add_argument('--low', type=int, default=600, help='低占空比阶跃(千分比)')
    ap.add_argument('--high', type=int, default=850, help='高占空比阶跃(千分比)')
    ap.add_argument('--duration', type=float, default=1.2, help='每次阶跃采样时长(s)')
    ap.add_argument('--ratio', type=float, default=1.5, help='闭环比开环快多少倍(越大越快/越易过冲)')
    ap.add_argument('--slew', type=int, default=500, help='参考爬升率(rpm/s, 越小越稳)')
    ap.add_argument('--fade', type=int, default=50, help='起步补偿衰减区间(rpm)')
    ap.add_argument('--csv', default=None, help='把曲线存到 CSV 文件')
    ap.add_argument('--plot', action='store_true', help='结束后用 matplotlib 弹窗画曲线(需先 pip install matplotlib)')
    ap.add_argument('--live', action='store_true', help='采样过程中实时滚动显示曲线(需先 pip install matplotlib)')
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    m = args.motor
    if not (0 < args.low < args.high <= 1000):
        print('错误: 需 0 < --low < --high <= 1000')
        return

    live = None
    if args.live:
        try:
            live = LivePlot()
        except ImportError:
            print('  未安装 matplotlib, 已跳过 --live: python -m pip install matplotlib')

    print('== 阶跃1: 占空比 %d ==' % args.low)
    s1 = capture_step(ser, m, args.low, args.duration, live, '阶跃1 占空比 %d' % args.low)
    A1, tau1, L1 = fit_first_order(s1)
    print('  稳态 %.1f rpm, tau=%.3fs, 死区时间 %.3fs' % (A1, tau1, L1))

    time.sleep(0.6)

    print('== 阶跃2: 占空比 %d ==' % args.high)
    s2 = capture_step(ser, m, args.high, args.duration, live, '阶跃2 占空比 %d' % args.high)
    A2, tau2, L2 = fit_first_order(s2)
    print('  稳态 %.1f rpm, tau=%.3fs, 死区时间 %.3fs' % (A2, tau2, L2))

    K = (A2 - A1) / (args.high - args.low)
    if K <= 1e-6:
        print('错误: 两次阶跃转速几乎相同, 辨识失败')
        return
    u0 = args.low - A1 / K

    # 闭环验证目标(偏中低转速), 据此把 tau 插值到实际工作点:
    #   低速段 stiction 使等效 tau 更大, 用工作点 tau 做极点对消才不会产生过冲
    target = max(60, int(A2 * 0.5))
    if A2 > A1:
        w = (target - A1) / (A2 - A1)
        if w < 0.0: w = 0.0
        elif w > 1.0: w = 1.0
        tau = tau1 + w * (tau2 - tau1)
    else:
        tau = tau2
    tau_cl = max(0.02, tau / args.ratio)

    ff = 1.0 / K
    kp = tau / (K * tau_cl)
    ki = 1.0 / (K * tau_cl)
    ff_offset = int(round(u0))
    stiction = int(round(u0 * 0.2))     # 超出死区的静摩擦额外部分
    stiction = max(50, min(300, stiction))

    print('== 辨识结果 ==')
    print('  K=%.4f rpm/千分比   死区 u0=%.0f   tau=%.3fs' % (K, u0, tau))
    print('  建议参数: ff=%.3f  kp=%.3f  ki=%.3f  ff_offset=%d  stiction=%d' % (ff, kp, ki, ff_offset, stiction))

    # 下发(广播所有电机, 可改成只发 --motor); 字段顺序必须与固件 SET_CTRL 一致: kp,ki,ff,ff_offset,stiction,slew,fade
    data = bytes([0xFF]) + struct.pack('<fffhhhh', kp, ki, ff, ff_offset, stiction, args.slew, args.fade)
    send(ser, SET_CTRL, data)
    print('  已通过 SET_CTRL 广播下发 ... %s' % ('OK' if read_frame(ser, 0.3) else '(无回应)'))

    # 闭环验证
    print('== 闭环验证: 目标 %d rpm ==' % target)
    send(ser, BRAKE, bytes([m]))          # 先刹停, 避免上一步开环滑行余速被当成"峰值"
    read_frame(ser, 0.15)
    time.sleep(0.4)
    send(ser, SET_SPEED, bytes([m]) + struct.pack('<h', target))
    read_frame(ser, 0.15)
    if live:
        live.begin('闭环验证 目标 %d rpm' % target, 2.0)
    vs = []
    t0 = time.time()
    while time.time() - t0 < 2.0:
        send(ser, GET_TEL)
        rpm = read_tel(ser, 0.05)
        if rpm is not None:
            t = time.time() - t0
            vs.append((t, float(rpm[m])))
            if live:
                live.add(t, float(rpm[m]))
    send(ser, BRAKE, bytes([m]))
    read_frame(ser, 0.15)

    if len(vs) >= 10:
        peak = max(r for _, r in vs)
        tail = sorted(r for _, r in vs[int(len(vs) * 0.7):])
        final = tail[len(tail) // 2]
        over = (peak - target) / target * 100.0
        print('  峰值 %.0f rpm, 稳态 %.0f rpm, 过冲 %.0f%%' % (peak, final, over))
        if over > 25:
            print('  过冲偏大 -> 调小 ff, 或调小 --ratio(让闭环更慢)')
        elif over < -5:
            print('  稳态偏低 -> 调大 ff')
        else:
            print('  响应良好, 可把参数写回 motor_service.c 的 MOTOR_DEFAULT_* 固化')

    steps = [('step1', s1), ('step2', s2), ('cl', vs)]
    if args.csv:
        save_csv(args.csv, steps)
        print('  曲线已存到 %s' % args.csv)

    if live and args.plot:
        live.close()                       # 关实时窗, 交给静态汇总图
    elif live:
        live.finish()                      # 阻塞保留实时窗, 直到手动关闭

    if args.plot:
        plot_steps(steps, '目标 %d rpm (CPR=1320)' % target)

    ser.close()


if __name__ == '__main__':
    main()
