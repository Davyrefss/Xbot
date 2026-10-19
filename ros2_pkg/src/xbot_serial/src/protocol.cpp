/**
 * @file protocol.cpp
 * @brief 二进制帧协议: 编解码 + CRC8 + 解析状态机
 * @note  与下位机 comm_protocol.c 逐字节等价, 改动时对照固件源码
 */

/* include files */
#include "xbot_serial/protocol.hpp"

#include <chrono>
#include <cstring>

namespace xbot_serial {
namespace protocol {

/* ================= 时间基准 ================= */

uint32_t monotonic_ms() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

/* ================= CRC8 ================= */

/**
 * @brief CRC8 单步更新
 * @note  多项式 0x31, 初值 0xFF, 无反射, 异或输出 0x00
 */
uint8_t crc8_update(uint8_t crc, uint8_t byte) {
    crc ^= byte;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ 0x31u)
                            : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

/**
 * @brief 计算一段数据的 CRC8
 */
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFFu;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_update(crc, data[i]);
    }
    return crc;
}

/* ================= 组帧 ================= */

/**
 * @brief 组帧
 * @param buf  输出缓冲(需 >= kMaxFrame 字节)
 * @param cmd  命令字
 * @param seq  帧序号
 * @param data 载荷
 * @param len  载荷长度
 * @return 帧总长度; len > kMaxPayload 时返回 0 不写入
 */
size_t build_frame(uint8_t *buf, uint8_t cmd, uint8_t seq,
                   const uint8_t *data, uint8_t len) {
    if (len > kMaxPayload) {
        return 0;
    }
    buf[0] = kSof;
    buf[1] = len;
    buf[2] = seq;
    buf[3] = cmd;
    if (len > 0) {
        std::memcpy(buf + 4, data, len);
    }
    /* CRC 覆盖 LEN+SEQ+CMD+DATA 共 (len+3) 字节, 不含帧头 */
    buf[4 + len] = crc8(buf + 1, static_cast<size_t>(len) + 3);
    return static_cast<size_t>(5) + len;
}

/* ================= 字节序打包/解包(小端) ================= */

void pack_i16(uint8_t *p, int16_t v) {
    uint16_t u = static_cast<uint16_t>(v);
    p[0] = static_cast<uint8_t>(u & 0xFFu);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFFu);
}

void pack_i32(uint8_t *p, int32_t v) {
    uint32_t u = static_cast<uint32_t>(v);
    p[0] = static_cast<uint8_t>(u & 0xFFu);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFFu);
    p[2] = static_cast<uint8_t>((u >> 16) & 0xFFu);
    p[3] = static_cast<uint8_t>((u >> 24) & 0xFFu);
}

/* 位模式与固件 union 写法相同, 用 memcpy 避免类型双关 */
void pack_f32(uint8_t *p, float v) {
    std::memcpy(p, &v, sizeof(v));
}

int16_t unpack_i16(const uint8_t *p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}

int32_t unpack_i32(const uint8_t *p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                                (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24));
}

float unpack_f32(const uint8_t *p) {
    float f = 0.0f;
    std::memcpy(&f, p, sizeof(f));
    return f;
}

/* ================= 解析状态机 ================= */

/**
 * @brief 初始化/复位解析状态(等价 comm_protocol_init)
 */
void ProtocolParser::reset() {
    state_ = State::kSof;
    crc_ = 0;
    idx_ = 0;
    last_activity_ms_ = 0;
}

/**
 * @brief 喂入 1 字节, 驱动解析状态机(等价 comm_protocol_feed)
 * @param byte   收到的字节
 * @param now_ms 当前单调毫秒
 * @return 解析出完整且 CRC 校验通过的帧返回 true, 否则 false
 */
bool ProtocolParser::feed(uint8_t byte, uint32_t now_ms) {
    last_activity_ms_ = now_ms;

    switch (state_) {
    case State::kSof:
        if (byte == kSof) {
            state_ = State::kLen;
            crc_ = 0xFFu;
        }
        break;

    case State::kLen:
        if (byte > kMaxPayload) {   /* 长度非法, 丢弃整帧重新找帧头 */
            state_ = State::kSof;
            break;
        }
        rx_.len = byte;
        crc_ = crc8_update(crc_, byte);
        state_ = State::kSeq;
        break;

    case State::kSeq:
        rx_.seq = byte;
        crc_ = crc8_update(crc_, byte);
        state_ = State::kCmd;
        break;

    case State::kCmd:
        rx_.cmd = byte;
        crc_ = crc8_update(crc_, byte);
        idx_ = 0;
        state_ = (rx_.len == 0) ? State::kCrc : State::kData;
        break;

    case State::kData:
        rx_.data[idx_++] = byte;
        crc_ = crc8_update(crc_, byte);
        if (idx_ >= rx_.len) {
            state_ = State::kCrc;
        }
        break;

    case State::kCrc:
        state_ = State::kSof;   /* 无论校验是否通过, 都回到等待帧头 */
        if (byte == crc_) {     /* CRC 校验通过 */
            parsed_ = rx_;
            return true;
        }
        break;

    default:
        state_ = State::kSof;
        break;
    }
    return false;
}

/**
 * @brief 帧超时复位(等价 comm_protocol_poll)
 * @note  帧中途长时间无数据(残留残缺帧)时复位, 避免后续帧被错误拼接;
 *        uint32 无符号减法, 回绕安全
 */
void ProtocolParser::poll(uint32_t now_ms) {
    if ((state_ != State::kSof) &&
        ((now_ms - last_activity_ms_) > kFrameTimeoutMs)) {
        state_ = State::kSof;
    }
}

/**
 * @brief 最近一次解析出的帧
 */
const Frame &ProtocolParser::get_frame() const {
    return parsed_;
}

} /* namespace protocol */
} /* namespace xbot_serial */
