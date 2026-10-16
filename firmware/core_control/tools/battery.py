#!/usr/bin/env python3
# 用法:
#   python tools/battery.py COM7        # 查询一次电量
#   python tools/battery.py COM7 5      # 每 5 秒轮询一次, Ctrl+C 停止
# 发送 GET_BATTERY(0x11), 解码 BATTERY(0x82) 应答并打印
import sys, time, struct, serial

SOF, GET_BAT, BAT = 0xA5, 0x11, 0x82


def crc8(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build(cmd, seq=0, data=b''):
    p = bytes([len(data), seq, cmd]) + data
    return bytes([SOF]) + p + bytes([crc8(p)])


def read_frame(ser, timeout=0.5):
    buf, t0 = bytearray(), time.time()
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b:
            continue
        buf.append(b[0])
        while buf and buf[0] != SOF:
            buf.pop(0)
        if len(buf) >= 5 and len(buf) >= 5 + buf[1]:
            f = bytes(buf[:5 + buf[1]])
            if crc8(f[1:4 + buf[1]]) == f[4 + buf[1]]:
                return f
            buf.pop(0)
    return None


def query(ser):
    ser.write(build(GET_BAT))
    f = read_frame(ser)
    if f is not None and f[3] == BAT and f[1] == 6:
        volt_v = struct.unpack('<H', f[4:6])[0] / 1000.0
        soc = f[6]
        mah = struct.unpack('<H', f[7:9])[0]
        low = '  [低电量!]' if f[9] & 0x01 else ''
        return '电压 %.2f V   电量 %3d%%   剩余 %4d mAh%s' % (volt_v, soc, mah, low)
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    loop_s = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    ser = serial.Serial(port, 115200, timeout=0.05)

    if loop_s <= 0:
        r = query(ser)
        print(r if r else '无回应/解析失败')
    else:
        print('每 %.0f 秒轮询, Ctrl+C 停止' % loop_s)
        try:
            while True:
                r = query(ser)
                print('%-8s %s' % (time.strftime('%H:%M:%S'), r if r else '无回应'))
                time.sleep(loop_s)
        except KeyboardInterrupt:
            pass
    ser.close()


if __name__ == '__main__':
    main()
