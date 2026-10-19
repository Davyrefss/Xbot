/**
 * @file protocol.hpp
 * @author Davy Refss
 * @brief 与下位机 comm_protocol.h/c 一致的二进制帧协议
 */

#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

/* include files */
#include <array>
#include <cstddef>
#include <cstdint>

namespace xbot_serial {
namespace protocol {

/** *****************************************************************************
 * 与下位机 comm_protocol.h/c 一致的二进制帧协议(小端字节序)
 *
 * 帧格式:
 *   | SOF(1) | LEN(1) | SEQ(1) | CMD(1) | DATA(LEN) | CRC8(1) |
 *
 *   SOF  : 帧头, 固定 0xA5
 *   LEN  : DATA 载荷字节数(0 ~ kMaxPayload)
 *   SEQ  : 帧序号, 每发一帧自增, 应答帧携带原帧 SEQ 用于对应
 *   CMD  : 命令字(见下方定义)
 *   DATA : 载荷, LEN 字节
 *   CRC8 : 对 LEN~DATA 共 (LEN+3) 字节校验; 多项式 0x31, 初值 0xFF, 无反射
 *  *****************************************************************************
 */
/* 常量 */
inline constexpr uint8_t kSof            = 0xA5u;
inline constexpr uint8_t kMaxPayload     = 32u;
inline constexpr uint8_t kMaxFrame       = kMaxPayload + 5u;   /* SOF+LEN+SEQ+CMD+DATA+CRC */
inline constexpr uint32_t kFrameTimeoutMs = 20u;               /* 帧超时(ms), 超时丢弃残缺帧 */
inline constexpr uint8_t kMotorNum       = 4u;

/* 下行命令(上位机 -> MCU) */
inline constexpr uint8_t kCmdPing         = 0x00u;   /* 心跳/探测, 无载荷 */
inline constexpr uint8_t kCmdSetSpeed     = 0x01u;   /* 单电机调速: [id:u8][rpm:i16] */
inline constexpr uint8_t kCmdSetAllSpeed  = 0x02u;   /* 四电机调速: [rpm1..rpm4:i16] */
inline constexpr uint8_t kCmdBrake        = 0x03u;   /* 单电机刹车: [id:u8] */
inline constexpr uint8_t kCmdBrakeAll     = 0x04u;   /* 全部刹车, 无载荷 */
inline constexpr uint8_t kCmdSetDuty      = 0x05u;   /* 单电机开环占空比: [id:u8][duty:i16] */
inline constexpr uint8_t kCmdSetCtrl      = 0x06u;   /* 控制参数: [id:u8][kp:f32][ki:f32][ff:f32][ff_offset:i16][stiction:i16][slew:i16][fade:i16], id=0xFF 广播 */
inline constexpr uint8_t kCmdGetTelemetry = 0x10u;   /* 请求一次遥测, 无载荷 */
inline constexpr uint8_t kCmdGetBattery   = 0x11u;   /* 请求一次电池信息, 无载荷 */

/* 上行命令(MCU -> 上位机) */
inline constexpr uint8_t kCmdAck          = 0x80u;   /* 应答: [ack_cmd:u8][result:u8] */
inline constexpr uint8_t kCmdTelemetry    = 0x81u;   /* 遥测: 4x speed_rpm[i16] + 4x position[i32] */
inline constexpr uint8_t kCmdBattery      = 0x82u;   /* 电池: voltage_mv[u16] + soc[u8] + capacity_mah[u16] + flags[u8] */

/* 应答结果码 */
inline constexpr uint8_t kResultOk        = 0x00u;   /* 成功 */
inline constexpr uint8_t kResultErrLen    = 0x01u;   /* 载荷长度错误 */
inline constexpr uint8_t kResultErrParam  = 0x02u;   /* 参数非法(如电机ID越界) */

/* 载荷长度 */
inline constexpr uint8_t kAckLen          = 2u;      /* [ack_cmd][result] */
inline constexpr uint8_t kTelemetryLen    = 24u;     /* 4*2(转速) + 4*4(位置) */
inline constexpr uint8_t kBatteryLen      = 6u;      /* 2(电压) + 1(电量) + 2(容量) + 1(标志) */
inline constexpr uint8_t kSetCtrlLen      = 21u;     /* 1 + 3*4 + 4*2 */
inline constexpr uint8_t kCtrlBroadcastId = 0xFFu;   /* SET_CTRL 广播电机 ID */

/* 电池标志位 */
inline constexpr uint8_t kBatteryFlagLow  = 0x01u;   /* bit0: 低电量 */

/* 解析出的帧结构 */
struct Frame {
    uint8_t cmd = 0;
    uint8_t seq = 0;
    uint8_t len = 0;
    std::array<uint8_t, kMaxPayload> data{};
};

/* 单调毫秒计数, uint32 回绕由无符号减法保证(等价 FreeRTOS tick 语义) */
uint32_t monotonic_ms();

/* CRC8 单步更新 */
uint8_t crc8_update(uint8_t crc, uint8_t byte);

/* 计算一段数据的 CRC8(多项式 0x31, 初值 0xFF, 无反射) */
uint8_t crc8(const uint8_t *data, size_t len);

/* 组帧: buf 需 >= kMaxFrame 字节, 返回帧总长度; len > kMaxPayload 返回 0 不写入 */
size_t build_frame(uint8_t *buf, uint8_t cmd, uint8_t seq,
                   const uint8_t *data, uint8_t len);

/* 字节序打包/解包(小端) */
void     pack_i16(uint8_t *p, int16_t v);
void     pack_i32(uint8_t *p, int32_t v);
void     pack_f32(uint8_t *p, float v);
int16_t  unpack_i16(const uint8_t *p);
int32_t  unpack_i32(const uint8_t *p);
float    unpack_f32(const uint8_t *p);

/* 解析状态机: 与下位机 comm_protocol.c 逐字节等价 */
class ProtocolParser {
public:
    /* 初始化/复位解析状态 */
    void reset();

    /* 喂入 1 字节, 解析出完整且 CRC 校验通过的帧返回 true, 否则 false */
    bool feed(uint8_t byte, uint32_t now_ms);

    /* 周期调用: 帧中途长时间无数据时复位解析状态(now_ms 为单调毫秒) */
    void poll(uint32_t now_ms);

    /* 最近一次解析出的帧(仅 feed 返回 true 后有效) */
    const Frame &get_frame() const;

private:
    enum class State {
        kSof = 0,   /* 等待帧头 0xA5 */
        kLen,       /* 读载荷长度 */
        kSeq,       /* 读帧序号 */
        kCmd,       /* 读命令字 */
        kData,      /* 读载荷 */
        kCrc,       /* 读并校验 CRC */
    };

    State    state_ = State::kSof;
    uint8_t  crc_ = 0;
    uint8_t  idx_ = 0;
    uint32_t last_activity_ms_ = 0;

    Frame rx_;      /* 正在组装的帧 */
    Frame parsed_;  /* 已完成校验的帧 */
};

} /* namespace protocol */
} /* namespace xbot_serial */

#endif /* PROTOCOL_HPP */
