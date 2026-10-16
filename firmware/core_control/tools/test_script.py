#!/usr/bin/env python3
# 用法: python test_script.py COM3
# 发送测试帧并打印 MCU 回包(已解码)
import sys, time, struct, serial

SOF = 0xA5
PING, SET_SPEED, SET_ALL, BRAKE, BRAKE_ALL, SET_DUTY, SET_CTRL, GET_TEL, GET_BAT = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x10, 0x11
ACK, TEL, BAT = 0x80, 0x81, 0x82

# 命令名 / 结果码
CMD_NAME = {
    0x00: "PING", 0x01: "SET_SPEED", 0x02: "SET_ALL_SPEED",
    0x03: "BRAKE", 0x04: "BRAKE_ALL", 0x05: "SET_DUTY", 0x06: "SET_CTRL",
    0x10: "GET_TELEMETRY", 0x11: "GET_BATTERY",
    0x80: "ACK", 0x81: "TELEMETRY", 0x82: "BATTERY",
}
RESULT_NAME = {0x00: "OK", 0x01: "ERR_LEN", 0x02: "ERR_PARAM"}


def crc8(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build(cmd, seq, data=b''):
    p = bytes([len(data), seq, cmd]) + data
    return bytes([SOF]) + p + bytes([crc8(p)])


def read_frame(ser, timeout=1.0):
    buf, t0 = bytearray(), time.time()
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b:
            continue
        buf.append(b[0])
        while buf and buf[0] != SOF:      # 丢到帧头
            buf.pop(0)
        if len(buf) >= 5 and len(buf) >= 5 + buf[1]:
            f = bytes(buf[:5 + buf[1]])
            if crc8(f[1:4 + buf[1]]) == f[4 + buf[1]]:
                return f
            buf.pop(0)                    # CRC 错: 丢 1 字节重找帧头
    return None


def print_frame(f):
    """解码并打印一帧"""
    if f is None:
        print("  (无回应)")
        return
    ln, seq, cmd = f[1], f[2], f[3]
    data = f[4:4 + ln]
    name = CMD_NAME.get(cmd, "0x%02X" % cmd)
    print("  帧: " + f.hex(' ').upper())
    print("     SOF=0x%02X  LEN=%d  SEQ=%d  CMD=%s(0x%02X)  DATA=%s  CRC=0x%02X"
          % (f[0], ln, seq, name, cmd, data.hex(' ').upper() or '-', f[4 + ln]))

    # 解码已知载荷
    if cmd == ACK and ln == 2:
        ack_cmd = CMD_NAME.get(data[0], "0x%02X" % data[0])
        print("     应答: 原命令=%s  结果=%s" % (ack_cmd, RESULT_NAME.get(data[1], "0x%02X" % data[1])))
    elif cmd == TEL and ln == 24:
        rpm = struct.unpack('<4h', data[0:8])
        pos = struct.unpack('<4i', data[8:24])
        print("     遥测: rpm = %s" % list(rpm))
        print("            pos = %s" % list(pos))
    elif cmd == BAT and ln == 6:
        volt_mv = struct.unpack('<H', data[0:2])[0]
        cap_mah = struct.unpack('<H', data[3:5])[0]
        low = '  [低电量!]' if data[5] & 0x01 else ''
        print("     电池: 电压 = %.2f V  电量 = %d%%  剩余容量 = %d mAh%s"
              % (volt_mv / 1000.0, data[2], cap_mah, low))


def send(ser, cmd, seq, data=b''):
    ser.write(build(cmd, seq, data))


def set_ctrl(ser, seq, mid, kp, ki, ff, ff_offset, stiction, slew, fade):
    """在线调参: mid=电机id(0~3) 或 0xFF=广播; 字段顺序 kp,ki,ff,ff_offset,stiction,slew,fade 须与固件一致"""
    data = bytes([mid]) + struct.pack('<fffhhhh', kp, ki, ff, ff_offset, stiction, slew, fade)
    send(ser, SET_CTRL, seq, data)


def get_battery(ser, seq):
    """请求一次电池信息, 返回 (电压V, 电量%, 剩余mAh, 低电量bool) 或 None"""
    send(ser, GET_BAT, seq)
    f = read_frame(ser)
    if f is not None and f[3] == BAT and f[1] == 6:
        volt_mv = struct.unpack('<H', f[4:6])[0]
        cap_mah = struct.unpack('<H', f[7:9])[0]
        return (volt_mv / 1000.0, f[6], cap_mah, bool(f[9] & 0x01))
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    ser = serial.Serial(port, 115200, timeout=0.05)

    # 1) 协议冒烟测试: 只验证命令/应答链路, 不驱动电机
    print('== 协议冒烟测试 ==')
    smoke = [
        ("心跳 PING",  PING,    bytes()),
        ("刹车 M1",    BRAKE,   bytes([0])),
        ("刹车 M2",    BRAKE,   bytes([1])),
        ("请求遥测",   GET_TEL, bytes()),
        ("请求电池",   GET_BAT, bytes()),
    ]
    for i, (label, cmd, data) in enumerate(smoke):
        send(ser, cmd, i, data)
        print("[%s]" % label)
        print_frame(read_frame(ser))

    # 2) 实际驱动测试: M1 目标 +300rpm 保持 3s, 轮询遥测看转速是否爬升
    print('\n== M1 目标 +300rpm (保持 3s) ==')
    send(ser, SET_SPEED, 20, bytes([0]) + struct.pack('<h', 300))
    print_frame(read_frame(ser))

    t0 = time.time()
    while time.time() - t0 < 3.0:
        send(ser, GET_TEL, 21)
        f = read_frame(ser)
        if f is not None and f[3] == TEL and f[1] == 24:
            rpm = struct.unpack('<4h', f[4:12])
            print('    t=%.1fs  rpm=%s' % (time.time() - t0, list(rpm)))
        time.sleep(0.2)

    send(ser, BRAKE, 22, bytes([0]))
    print_frame(read_frame(ser))
    print('    M1 已刹车')

    ser.close()


if __name__ == '__main__':
    main()
