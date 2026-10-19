/**
 * @file protocol_test.cpp
 * @brief 协议模块自检: golden 帧向量 + 状态机边界用例
 * @note  不引 gtest, assert 风格, 0/1 返回码, 注册进 ctest
 * @note  golden 向量由与固件同算法的独立 Python 实现生成,
 *        任何与固件 comm_protocol.c 的等价性回归都会导致断言失败
 */

#include "xbot_serial/protocol.hpp"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace xbot_serial::protocol;

/* ================= 辅助 ================= */

static int g_failures = 0;

#define CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
            g_failures++; \
        } \
    } while (0)

/* 把 hex 字符串("A5 00 00 00 4B")转成字节向量 */
static std::vector<uint8_t> hex_vec(const char *s) {
    std::vector<uint8_t> v;
    unsigned int b;
    while (*s != '\0' && std::sscanf(s, "%x", &b) == 1) {
        v.push_back(static_cast<uint8_t>(b));
        while (*s != '\0' && *s != ' ') s++;
        while (*s == ' ') s++;
    }
    return v;
}

static bool frame_eq(const uint8_t *got, size_t got_len, const char *hex) {
    std::vector<uint8_t> want = hex_vec(hex);
    if (got_len != want.size()) {
        return false;
    }
    return std::memcmp(got, want.data(), want.size()) == 0;
}

/* ================= CRC8 ================= */

static void test_crc8() {
    uint8_t a[3] = { 0x00, 0x00, 0x00 };
    CHECK(crc8(a, 3) == 0x4Bu, "crc8({0,0,0}) != 0x4B");
    CHECK(crc8(nullptr, 0) == 0xFFu, "crc8(empty) != 0xFF");

    /* 固件已知帧头序列: LEN=0 SEQ=0 CMD=0 的 CRC */
    uint8_t head[3] = { 0x00, 0x00, 0x00 };
    CHECK(crc8(head, 3) == 0x4Bu, "PING header crc");
}

/* ================= 组帧 golden 向量 ================= */

static void test_build_frame_golden() {
    uint8_t buf[kMaxFrame];
    size_t n;

    /* PING (seq=0) */
    n = build_frame(buf, kCmdPing, 0, nullptr, 0);
    CHECK(frame_eq(buf, n, "A5 00 00 00 4B"), "PING golden");

    /* SET_SPEED id=1 rpm=1500 (seq=42) */
    uint8_t p3[3] = { 1, 0xDC, 0x05 };
    n = build_frame(buf, kCmdSetSpeed, 42, p3, 3);
    CHECK(frame_eq(buf, n, "A5 03 2A 01 01 DC 05 9A"), "SET_SPEED golden");

    /* SET_ALL_SPEED 100,-100,200,-200 (seq=1) */
    uint8_t p8[8];
    pack_i16(&p8[0], 100);
    pack_i16(&p8[2], -100);
    pack_i16(&p8[4], 200);
    pack_i16(&p8[6], -200);
    n = build_frame(buf, kCmdSetAllSpeed, 1, p8, 8);
    CHECK(frame_eq(buf, n, "A5 08 01 02 64 00 9C FF C8 00 38 FF 93"),
          "SET_ALL_SPEED golden");

    /* BRAKE id=3 */
    uint8_t p1[1] = { 3 };
    n = build_frame(buf, kCmdBrake, 0, p1, 1);
    CHECK(frame_eq(buf, n, "A5 01 00 03 03 32"), "BRAKE golden");

    /* BRAKE_ALL */
    n = build_frame(buf, kCmdBrakeAll, 0, nullptr, 0);
    CHECK(frame_eq(buf, n, "A5 00 00 04 8F"), "BRAKE_ALL golden");

    /* SET_DUTY id=1 duty=-1000 */
    uint8_t pd[3] = { 1, 0, 0 };
    pack_i16(&pd[1], -1000);
    n = build_frame(buf, kCmdSetDuty, 1, pd, 3);
    CHECK(frame_eq(buf, n, "A5 03 01 05 01 18 FC 35"), "SET_DUTY golden");

    /* SET_CTRL id=255 kp=1.0 ki=0.1 ff=0.05 100,50,300,200 (seq=7) */
    uint8_t pc[21];
    pc[0] = kCtrlBroadcastId;
    pack_f32(&pc[1], 1.0f);
    pack_f32(&pc[5], 0.1f);
    pack_f32(&pc[9], 0.05f);
    pack_i16(&pc[13], 100);
    pack_i16(&pc[15], 50);
    pack_i16(&pc[17], 300);
    pack_i16(&pc[19], 200);
    n = build_frame(buf, kCmdSetCtrl, 7, pc, 21);
    CHECK(frame_eq(buf, n,
        "A5 15 07 06 FF 00 00 80 3F CD CC CC 3D CD CC 4C 3D "
        "64 00 32 00 2C 01 C8 00 01"), "SET_CTRL golden");

    /* GET_TELEMETRY / GET_BATTERY */
    n = build_frame(buf, kCmdGetTelemetry, 0, nullptr, 0);
    CHECK(frame_eq(buf, n, "A5 00 00 10 08"), "GET_TELEMETRY golden");
    n = build_frame(buf, kCmdGetBattery, 0, nullptr, 0);
    CHECK(frame_eq(buf, n, "A5 00 00 11 39"), "GET_BATTERY golden");

    /* ACK cmd=0x02 result=OK seq=5 */
    uint8_t pa[2] = { kCmdSetAllSpeed, kResultOk };
    n = build_frame(buf, kCmdAck, 5, pa, 2);
    CHECK(frame_eq(buf, n, "A5 02 05 80 02 00 9D"), "ACK golden");

    /* TELEMETRY: 上行帧同样用 build_frame 组帧(测试/模拟下位机用) */
    uint8_t pt[24];
    pack_i16(&pt[0], 100);
    pack_i16(&pt[2], 200);
    pack_i16(&pt[4], -300);
    pack_i16(&pt[6], 0);
    pack_i32(&pt[8], 1);
    pack_i32(&pt[12], -2);
    pack_i32(&pt[16], 300000);
    pack_i32(&pt[20], -400000);
    n = build_frame(buf, kCmdTelemetry, 9, pt, 24);
    CHECK(frame_eq(buf, n,
        "A5 18 09 81 64 00 C8 00 D4 FE 00 00 01 00 00 00 "
        "FE FF FF FF E0 93 04 00 80 E5 F9 FF D6"), "TELEMETRY golden");

    /* BATTERY 12400mV 85% 4300mAh flags=0 */
    uint8_t pb[6] = { 0x70, 0x30, 85, 0xCC, 0x10, 0 };
    n = build_frame(buf, kCmdBattery, 3, pb, 6);
    CHECK(frame_eq(buf, n, "A5 06 03 82 70 30 55 CC 10 00 0F"), "BATTERY golden");

    /* 32B 满载荷: SOF + LEN=0x20 + SEQ=0 + CMD=0x06 + data[0..31] + CRC */
    uint8_t full[32];
    for (uint8_t i = 0; i < 32; i++) {
        full[i] = i;
    }
    n = build_frame(buf, 0x06, 0, full, 32);
    CHECK(frame_eq(buf, n,
        "A5 20 00 06 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F "
        "10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F CD"),
        "32B payload golden");

    /* 非法长度: > kMaxPayload 返回 0 且不写入 */
    n = build_frame(buf, 0x00, 0, full, 33);
    CHECK(n == 0, "build_frame len=33 should return 0");
}

/* ================= 字节序打包/解包 ================= */

static void test_pack_unpack() {
    uint8_t b2[2];
    uint8_t b4[4];

    pack_i16(b2, 0);
    CHECK(unpack_i16(b2) == 0, "i16 roundtrip 0");
    pack_i16(b2, 1);
    CHECK(unpack_i16(b2) == 1, "i16 roundtrip 1");
    pack_i16(b2, -1);
    CHECK(unpack_i16(b2) == -1, "i16 roundtrip -1");
    pack_i16(b2, 32767);
    CHECK(unpack_i16(b2) == 32767, "i16 roundtrip 32767");
    pack_i16(b2, -32768);
    CHECK(unpack_i16(b2) == -32768, "i16 roundtrip -32768");

    pack_i32(b4, 0);
    CHECK(unpack_i32(b4) == 0, "i32 roundtrip 0");
    pack_i32(b4, -1);
    CHECK(unpack_i32(b4) == -1, "i32 roundtrip -1");
    pack_i32(b4, 2147483647);
    CHECK(unpack_i32(b4) == 2147483647, "i32 roundtrip INT32_MAX");
    pack_i32(b4, -2147483647 - 1);
    CHECK(unpack_i32(b4) == -2147483647 - 1, "i32 roundtrip INT32_MIN");

    pack_f32(b4, 1.5f);
    CHECK(unpack_f32(b4) == 1.5f, "f32 roundtrip 1.5");
    pack_f32(b4, 0.1f);
    CHECK(unpack_f32(b4) == 0.1f, "f32 roundtrip 0.1");
    pack_f32(b4, -3.25f);
    CHECK(unpack_f32(b4) == -3.25f, "f32 roundtrip -3.25");
}

/* ================= 解析状态机 ================= */

/* 喂一整个字节流, 收集解析出的帧数 */
static int feed_all(ProtocolParser &parser, const std::vector<uint8_t> &bytes,
                    uint32_t &now_ms) {
    int frames = 0;
    for (uint8_t b : bytes) {
        if (parser.feed(b, now_ms)) {
            frames++;
        }
    }
    return frames;
}

static void test_parser() {
    uint32_t now = 1000;
    ProtocolParser parser;

    /* 1. 正常单帧: PING */
    parser.reset();
    now += 10;
    std::vector<uint8_t> ping = hex_vec("A5 00 00 00 4B");
    CHECK(feed_all(parser, ping, now) == 1, "parse single PING");
    CHECK(parser.get_frame().cmd == kCmdPing, "PING cmd");
    CHECK(parser.get_frame().seq == 0, "PING seq");
    CHECK(parser.get_frame().len == 0, "PING len");

    /* 2. 带载荷帧: SET_ALL_SPEED */
    parser.reset();
    now += 10;
    std::vector<uint8_t> all = hex_vec("A5 08 01 02 64 00 9C FF C8 00 38 FF 93");
    CHECK(feed_all(parser, all, now) == 1, "parse SET_ALL_SPEED");
    CHECK(parser.get_frame().cmd == kCmdSetAllSpeed, "SET_ALL_SPEED cmd");
    CHECK(parser.get_frame().len == 8, "SET_ALL_SPEED len");
    CHECK(unpack_i16(&parser.get_frame().data[0]) == 100, "rpm1");
    CHECK(unpack_i16(&parser.get_frame().data[2]) == -100, "rpm2");
    CHECK(unpack_i16(&parser.get_frame().data[4]) == 200, "rpm3");
    CHECK(unpack_i16(&parser.get_frame().data[6]) == -200, "rpm4");

    /* 3. 32B 满载荷帧 */
    parser.reset();
    now += 10;
    std::vector<uint8_t> full = hex_vec(
        "A5 20 00 06 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F "
        "10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F CD");
    CHECK(feed_all(parser, full, now) == 1, "parse 32B payload");
    CHECK(parser.get_frame().len == 32, "32B len");
    CHECK(parser.get_frame().data[31] == 0x1F, "32B last byte");

    /* 4. 背靠背两帧: PING(seq=0) + BRAKE_ALL(seq=1) */
    parser.reset();
    now += 10;
    std::vector<uint8_t> two = hex_vec("A5 00 00 00 4B A5 00 01 04 7B");
    CHECK(feed_all(parser, two, now) == 2, "parse two back-to-back frames");
    CHECK(parser.get_frame().cmd == kCmdBrakeAll, "second frame cmd");

    /* 5. CRC 错误帧被丢弃, 随后完整帧正常解析 */
    parser.reset();
    now += 10;
    std::vector<uint8_t> bad = hex_vec("A5 00 03 00 99");  /* CRC 应为 0x66 */
    CHECK(feed_all(parser, bad, now) == 0, "bad CRC frame dropped");
    std::vector<uint8_t> good = hex_vec("A5 00 03 00 66");
    CHECK(feed_all(parser, good, now) == 1, "good frame after bad CRC");
    CHECK(parser.get_frame().seq == 3, "recovered frame seq");

    /* 6. LEN=33 非法长度回 SOF, 后续帧正常 */
    parser.reset();
    now += 10;
    std::vector<uint8_t> badlen = hex_vec("A5 21 00 00 00");  /* LEN=33 > 32 */
    CHECK(feed_all(parser, badlen, now) == 0, "invalid LEN dropped");
    CHECK(feed_all(parser, ping, now) == 1, "frame after invalid LEN");

    /* 7. DATA 内嵌 0xA5 不得被当成新帧头 */
    parser.reset();
    now += 10;
    std::vector<uint8_t> embedded = hex_vec(
        "A5 08 02 02 42 A5 42 42 42 42 42 42 9E");
    CHECK(feed_all(parser, embedded, now) == 1, "embedded 0xA5 not a new SOF");
    CHECK(parser.get_frame().data[1] == 0xA5, "embedded 0xA5 byte kept");

    /* 8. 残缺帧 + poll 超时复位后, 完整帧可解析 */
    parser.reset();
    now += 10;
    std::vector<uint8_t> partial = hex_vec("A5 08 01 02 64 00");  /* 缺 6 字节 */
    CHECK(feed_all(parser, partial, now) == 0, "partial frame no result");
    now += 100;  /* 超过 20ms 帧超时 */
    parser.poll(now);
    CHECK(feed_all(parser, all, now) == 1, "full frame after poll reset");

    /* 9. poll 在帧超时前不复位: 帧内间隙 < 20ms 应继续拼接 */
    parser.reset();
    now += 10;
    std::vector<uint8_t> p1 = hex_vec("A5 08 01 02 64 00");
    std::vector<uint8_t> p2 = hex_vec("9C FF C8 00 38 FF 93");
    CHECK(feed_all(parser, p1, now) == 0, "first half no result");
    now += 5;  /* 未超时 */
    parser.poll(now);
    CHECK(feed_all(parser, p2, now) == 1, "second half completes frame");

    /* 10. feed 返回 false 时 get_frame 保持上一帧 */
    parser.reset();
    now += 10;
    CHECK(feed_all(parser, ping, now) == 1, "seed frame");
    const Frame &before = parser.get_frame();
    std::vector<uint8_t> junk = { 0x00, 0x5A, 0xA5 };
    CHECK(feed_all(parser, junk, now) == 0, "junk bytes no frame");
    CHECK(parser.get_frame().seq == before.seq &&
          parser.get_frame().cmd == before.cmd,
          "get_frame keeps last parsed frame");

    /* 11. reset 后残留状态清空: 缺 CRC 的帧后 reset, 再喂完整帧 */
    parser.reset();
    now += 10;
    std::vector<uint8_t> nocrc = hex_vec("A5 00 05 00");  /* 缺 CRC */
    CHECK(feed_all(parser, nocrc, now) == 0, "no crc no frame");
    parser.reset();
    CHECK(feed_all(parser, ping, now) == 1, "frame after reset");
}

int main() {
    test_crc8();
    test_build_frame_golden();
    test_pack_unpack();
    test_parser();

    if (g_failures == 0) {
        std::printf("protocol_test: all checks passed\n");
        return 0;
    }
    std::printf("protocol_test: %d check(s) failed\n", g_failures);
    return 1;
}
