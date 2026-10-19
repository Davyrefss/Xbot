#!/usr/bin/env python3
"""模拟 STM32 下位机(无硬件联调用)

协议与 firmware/core_control/source/service/comm_protocol.c 一致:
  帧格式: 0xA5 | LEN | SEQ | CMD | DATA(LEN) | CRC8
  CRC8: 多项式 0x31, 初值 0xFF, 无反射; 覆盖 LEN~DATA 共 LEN+3 字节
  多字节字段为小端

功能:
  1. 接收上位机指令帧(PING/SET_SPEED/SET_ALL_SPEED/BRAKE/BRAKE_ALL/
     SET_DUTY/SET_CTRL/GET_TELEMETRY/GET_BATTERY),
     校验 CRC8, 回 ACK 应答, 打印解析结果(行为对照 app/comm_task.c)
  2. 按 20Hz(50ms) 回传遥测帧: 4x 转速(RPM) + 4x 累计位置(编码器计数);
     加 --no-auto-telemetry 可关闭主动上报, 贴近真机(固件只在收到
     GET_TELEMETRY 请求时才回遥测)
  3. 轮速按一阶滞后跟踪指令转速, 模拟电机响应过程
     注意: SET_SPEED/SET_ALL_SPEED 载荷为转速 RPM(固件按闭环 RPM 处理);
     SET_DUTY 载荷为占空比(-1000~1000), 按 MAX_RPM 折算成转速仿真

用法(三个终端):
  # 终端1: 创建虚拟串口对
  socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
  # 终端2: 运行模拟下位机
  python3 src/xbot_serial/scripts/mock_stm32.py /tmp/ttyV1
  # 终端3: 运行 ROS 节点
  ros2 run xbot_serial manual_sender --ros-args -p port:=/tmp/ttyV0
"""

import argparse
import os
import select
import struct
import sys
import termios
import time

SOF = 0xA5
MAX_PAYLOAD = 32
MAX_FRAME = MAX_PAYLOAD + 5

CMD_PING = 0x00          # 心跳, 无载荷
CMD_SET_SPEED = 0x01     # 单电机调速: [id:u8][rpm:i16](固件闭环 RPM)
CMD_SET_ALL_SPEED = 0x02  # 四电机调速: [rpm1..rpm4:i16](固件闭环 RPM)
CMD_BRAKE = 0x03         # 单电机刹车: [id:u8]
CMD_BRAKE_ALL = 0x04     # 全部刹车, 无载荷
CMD_SET_DUTY = 0x05      # 单电机开环占空比: [id:u8][duty:i16]
CMD_SET_CTRL = 0x06      # 控制参数: [id][kp:f32][ki:f32][ff:f32][ff_offset:i16]
                          #           [stiction:i16][slew:i16][fade:i16], id=0xFF 广播
CMD_GET_TELEMETRY = 0x10  # 请求一次遥测, 无载荷
CMD_GET_BATTERY = 0x11   # 请求一次电池信息, 无载荷
CMD_ACK = 0x80           # 应答: [ack_cmd:u8][result:u8]
CMD_TELEMETRY = 0x81     # 遥测: 4x rpm[i16] + 4x pos[i32]
CMD_BATTERY = 0x82       # 电池: voltage_mv[u16] + soc[u8] + capacity_mah[u16] + flags[u8]

RESULT_OK = 0
RESULT_ERR_LEN = 1
RESULT_ERR_PARAM = 2

MOTOR_NUM = 4
ENC_CPR = 44             # 编码器每圈计数, 与下位机 MOTOR_ENC_CPR 一致
MAX_RPM = 300.0          # 满占空比转速 RPM, 仅用于 SET_DUTY 仿真折算
TELEMETRY_PERIOD = 0.05  # 遥测周期: 50ms(20Hz), 与下位机 comm_task 一致


def crc8(data: bytes) -> int:
    """CRC8, 多项式 0x31, 初值 0xFF, 无反射, 与 comm_protocol.c 保持一致"""
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, seq: int, payload: bytes = b"") -> bytes:
    """组帧: 帧头 + LEN + SEQ + CMD + 负载 + CRC8"""
    head = bytes([SOF, len(payload), seq & 0xFF, cmd]) + payload
    return head + bytes([crc8(head[1:])])   # CRC 覆盖 LEN~DATA, 不含帧头


def pack_i16(v: int) -> bytes:
    return int(v).to_bytes(2, "little", signed=True)


def pack_i32(v: int) -> bytes:
    return int(v).to_bytes(4, "little", signed=True)


def unpack_i16(p: bytes) -> int:
    return int.from_bytes(p, "little", signed=True)


def unpack_f32(p: bytes) -> float:
    """解包 f32(小端, IEEE754), 位模式与固件 comm_unpack_f32 一致"""
    return struct.unpack("<f", p)[0]


class FrameParser:
    """与 comm_protocol.c 中解析状态机等价的 Python 实现"""

    def __init__(self):
        self.state = "SOF"
        self.length = 0
        self.seq = 0
        self.cmd = 0
        self.payload = bytearray()
        self.crc = 0

    def feed(self, data: bytes):
        """喂入字节流, 返回本次解析出的帧列表 [(cmd, seq, payload), ...]"""
        frames = []
        for b in data:
            frame = self._feed_byte(b)
            if frame is not None:
                frames.append(frame)
        return frames

    def _feed_byte(self, b: int):
        if self.state == "SOF":
            if b == SOF:
                self.crc = 0xFF
                self.state = "LEN"
        elif self.state == "LEN":
            if b > MAX_PAYLOAD:
                self.state = "SOF"      # 长度非法, 丢弃整帧
                return None
            self.length = b
            self.crc = self._crc_step(self.crc, b)
            self.state = "SEQ"
        elif self.state == "SEQ":
            self.seq = b
            self.crc = self._crc_step(self.crc, b)
            self.state = "CMD"
        elif self.state == "CMD":
            self.cmd = b
            self.crc = self._crc_step(self.crc, b)
            self.payload = bytearray()
            self.state = "DATA" if self.length else "CRC"
        elif self.state == "DATA":
            self.payload.append(b)
            self.crc = self._crc_step(self.crc, b)
            if len(self.payload) >= self.length:
                self.state = "CRC"
        elif self.state == "CRC":
            self.state = "SOF"
            if b == self.crc:
                return (self.cmd, self.seq, bytes(self.payload))
        return None

    @staticmethod
    def _crc_step(crc: int, b: int) -> int:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        return crc


def main():
    parser = argparse.ArgumentParser(description="模拟 STM32 下位机")
    parser.add_argument("port", help="虚拟串口设备, 例如 /tmp/ttyV1")
    parser.add_argument("--no-auto-telemetry", action="store_true",
                        help="关闭 20Hz 主动遥测上报, 贴近真机(仅响应 GET_TELEMETRY)")
    args = parser.parse_args()

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY)
    # pty 默认是 cooked 模式, 会改写 0x0A 等二进制字节, 必须切成 raw 模式
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                            # iflag 清零: 关闭输入转换
    attrs[1] &= ~termios.OPOST              # oflag: 关闭输出处理
    attrs[3] &= ~(termios.ICANON | termios.ECHO)  # lflag: raw 输入
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    frame_parser = FrameParser()
    seq = 0
    target_rpm = [0.0] * MOTOR_NUM   # 指令目标轮速(由占空比折算)
    actual_rpm = [0.0] * MOTOR_NUM   # 模拟的实际轮速(一阶滞后)
    pos_count = [0] * MOTOR_NUM      # 累计位置(编码器计数)

    last_telemetry = time.monotonic()
    last_print = time.monotonic()

    def send_ack(ack_cmd: int, result: int, rx_seq: int):
        os.write(fd, build_frame(CMD_ACK, rx_seq, bytes([ack_cmd, result])))

    def send_telemetry(tx_seq: int):
        payload = b"".join(pack_i16(round(r)) for r in actual_rpm)
        payload += b"".join(pack_i32(round(p)) for p in pos_count)
        os.write(fd, build_frame(CMD_TELEMETRY, tx_seq, payload))

    def send_battery(tx_seq: int):
        """模拟电池: 电压 12400mV, 电量 85%, 剩余容量 4300mAh, 无标志位"""
        payload = pack_i16(12400) + bytes([85]) + pack_i16(4300) + bytes([0])
        os.write(fd, build_frame(CMD_BATTERY, tx_seq, payload))

    print(f"mock STM32 started on {args.port}, waiting for commands...", flush=True)

    while True:
        # 读串口(最多等 10ms, 不阻塞速度模拟)
        ready, _, _ = select.select([fd], [], [], 0.01)
        if ready:
            data = os.read(fd, 256)
            if not data:
                print("serial port closed, exit", flush=True)
                break
            # 分发逻辑对照固件 app/comm_task.c 的 handle_frame
            for cmd, rx_seq, payload in frame_parser.feed(data):
                if cmd == CMD_PING:
                    send_ack(cmd, RESULT_OK, rx_seq)
                elif cmd == CMD_SET_SPEED:
                    if len(payload) != 3:
                        send_ack(cmd, RESULT_ERR_LEN, rx_seq)
                        continue
                    m_id, rpm = payload[0], unpack_i16(payload[1:])
                    if m_id >= MOTOR_NUM:
                        send_ack(cmd, RESULT_ERR_PARAM, rx_seq)
                    else:
                        target_rpm[m_id] = rpm
                        send_ack(cmd, RESULT_OK, rx_seq)
                elif cmd == CMD_SET_ALL_SPEED:
                    if len(payload) != 8:
                        send_ack(cmd, RESULT_ERR_LEN, rx_seq)
                        continue
                    rpm = [unpack_i16(payload[2 * i:2 * i + 2])
                           for i in range(MOTOR_NUM)]
                    target_rpm = [float(r) for r in rpm]
                    send_ack(cmd, RESULT_OK, rx_seq)
                    print(f"[RX] SET_ALL_SPEED: rpm={rpm}", flush=True)
                elif cmd == CMD_BRAKE:
                    if len(payload) != 1:
                        send_ack(cmd, RESULT_ERR_LEN, rx_seq)
                        continue
                    m_id = payload[0]
                    if m_id >= MOTOR_NUM:
                        send_ack(cmd, RESULT_ERR_PARAM, rx_seq)
                    else:
                        target_rpm[m_id] = 0.0
                        send_ack(cmd, RESULT_OK, rx_seq)
                elif cmd == CMD_BRAKE_ALL:
                    target_rpm = [0.0] * MOTOR_NUM
                    send_ack(cmd, RESULT_OK, rx_seq)
                    print("[RX] BRAKE_ALL", flush=True)
                elif cmd == CMD_SET_DUTY:
                    if len(payload) != 3:
                        send_ack(cmd, RESULT_ERR_LEN, rx_seq)
                        continue
                    m_id, duty = payload[0], unpack_i16(payload[1:])
                    if m_id >= MOTOR_NUM:
                        send_ack(cmd, RESULT_ERR_PARAM, rx_seq)
                    else:
                        target_rpm[m_id] = duty / 1000.0 * MAX_RPM
                        send_ack(cmd, RESULT_OK, rx_seq)
                elif cmd == CMD_SET_CTRL:
                    if len(payload) != 21:
                        send_ack(cmd, RESULT_ERR_LEN, rx_seq)
                        continue
                    m_id = payload[0]
                    if m_id != 0xFF and m_id >= MOTOR_NUM:
                        send_ack(cmd, RESULT_ERR_PARAM, rx_seq)
                        continue
                    kp = unpack_f32(payload[1:5])
                    ki = unpack_f32(payload[5:9])
                    ff = unpack_f32(payload[9:13])
                    ff_offset = unpack_i16(payload[13:15])
                    stiction = unpack_i16(payload[15:17])
                    slew = unpack_i16(payload[17:19])
                    fade = unpack_i16(payload[19:21])
                    send_ack(cmd, RESULT_OK, rx_seq)
                    print(f"[RX] SET_CTRL: id={m_id} kp={kp} ki={ki} ff={ff} "
                          f"ff_offset={ff_offset} stiction={stiction} "
                          f"slew={slew} fade={fade}", flush=True)
                elif cmd == CMD_GET_TELEMETRY:
                    send_telemetry(rx_seq)
                elif cmd == CMD_GET_BATTERY:
                    send_battery(rx_seq)
                else:
                    print(f"[RX] unknown cmd 0x{cmd:02X} len={len(payload)}", flush=True)

        now = time.monotonic()
        dt = 0.01   # 主循环节拍近似 10ms

        # 一阶滞后, 模拟电机加速过程; 同时积分累计位置(44 计数/圈)
        for i in range(MOTOR_NUM):
            actual_rpm[i] += 0.2 * (target_rpm[i] - actual_rpm[i])
            pos_count[i] += actual_rpm[i] / 60.0 * ENC_CPR * dt

        # 20Hz 回传遥测(真机固件不主动上报, --no-auto-telemetry 可关闭)
        if not args.no_auto_telemetry and now - last_telemetry >= TELEMETRY_PERIOD:
            last_telemetry = now
            send_telemetry(seq)
            seq += 1

        # 2Hz 打印状态
        if now - last_print >= 0.5:
            last_print = now
            print(f"[TX] telemetry: rpm={[round(r) for r in actual_rpm]} "
                  f"pos={[round(p) for p in pos_count]}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
