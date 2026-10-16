/**
 * @file comm_protocol.c
 * @brief 二进制帧协议: 编解码 + CRC8 + 解析状态机
 */

#include "comm_protocol.h"

/* 解析状态机 */
typedef enum {
    PARSE_SOF = 0,   /* 等待帧头 0xA5 */
    PARSE_LEN,       /* 读载荷长度 */
    PARSE_SEQ,       /* 读帧序号 */
    PARSE_CMD,       /* 读命令字 */
    PARSE_DATA,      /* 读载荷 */
    PARSE_CRC,       /* 读并校验 CRC */
} parse_state_t;

/* static variables */
static parse_state_t parse_state = PARSE_SOF;
static uint8_t  parse_crc = 0;
static uint8_t  parse_idx = 0;
static uint32_t last_activity_tick = 0;

static comm_frame_t rx_frame;      /* 正在组装的帧 */
static comm_frame_t parsed_frame;  /* 已完成校验的帧 */

/**
 * @brief CRC8 单步更新
 * @note  多项式 0x31, 初值 0xFF, 无反射, 异或输出 0x00
 */
static uint8_t crc8_update(uint8_t crc, uint8_t byte){
    crc ^= byte;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
    }
    return crc;
}

/**
 * @brief 计算一段数据的 CRC8
 */
static uint8_t crc8(const uint8_t *data, uint16_t len){
    uint8_t crc = 0xFFu;
    while (len--) {
        crc = crc8_update(crc, *data++);
    }
    return crc;
}

/**
 * @brief 协议初始化
 */
void comm_protocol_init(void){
    parse_state = PARSE_SOF;
    parse_crc = 0;
    parse_idx = 0;
    last_activity_tick = 0;
}

/**
 * @brief 喂入 1 字节, 驱动解析状态机
 * @param byte 收到的字节
 * @param now_tick 当前 FreeRTOS tick(1 tick=1ms)
 * @return 解析出完整且 CRC 校验通过的帧返回 1, 否则 0
 */
int comm_protocol_feed(uint8_t byte, uint32_t now_tick){
    last_activity_tick = now_tick;

    switch (parse_state) {
        case PARSE_SOF:
            if (byte == COMM_SOF) {
                parse_state = PARSE_LEN;
                parse_crc = 0xFFu;
            }
            break;

        case PARSE_LEN:
            if (byte > COMM_MAX_PAYLOAD) {   /* 长度非法, 丢弃整帧重新找帧头 */
                parse_state = PARSE_SOF;
                break;
            }
            rx_frame.len = byte;
            parse_crc = crc8_update(parse_crc, byte);
            parse_state = PARSE_SEQ;
            break;

        case PARSE_SEQ:
            rx_frame.seq = byte;
            parse_crc = crc8_update(parse_crc, byte);
            parse_state = PARSE_CMD;
            break;

        case PARSE_CMD:
            rx_frame.cmd = byte;
            parse_crc = crc8_update(parse_crc, byte);
            parse_idx = 0;
            parse_state = (rx_frame.len == 0) ? PARSE_CRC : PARSE_DATA;
            break;

        case PARSE_DATA:
            rx_frame.data[parse_idx++] = byte;
            parse_crc = crc8_update(parse_crc, byte);
            if (parse_idx >= rx_frame.len) {
                parse_state = PARSE_CRC;
            }
            break;

        case PARSE_CRC:
            parse_state = PARSE_SOF;
            if (byte == parse_crc) {   /* CRC 校验通过 */
                parsed_frame = rx_frame;
                return 1;
            }
            break;

        default:
            parse_state = PARSE_SOF;
            break;
    }
    return 0;
}

/**
 * @brief 帧超时复位
 * @note  帧中途长时间无数据(残留残缺帧)时复位, 避免后续帧被错误拼接
 */
void comm_protocol_poll(uint32_t now_tick){
    if ((parse_state != PARSE_SOF) &&
        ((now_tick - last_activity_tick) > COMM_FRAME_TIMEOUT_MS)) {
        parse_state = PARSE_SOF;
    }
}

/**
 * @brief 最近一次解析出的帧
 */
const comm_frame_t* comm_protocol_get_frame(void){
    return &parsed_frame;
}

/**
 * @brief 组帧
 * @param buf 输出缓冲(需 >= COMM_MAX_FRAME 字节)
 * @param cmd 命令字
 * @param seq 帧序号
 * @param data 载荷
 * @param len 载荷长度
 * @return 帧总长度
 */
uint16_t comm_frame_build(uint8_t *buf, uint8_t cmd, uint8_t seq,
                          const uint8_t *data, uint8_t len){
    buf[0] = COMM_SOF;
    buf[1] = len;
    buf[2] = seq;
    buf[3] = cmd;
    for (uint8_t i = 0; i < len; i++) {
        buf[4 + i] = data[i];
    }
    /* CRC 覆盖 LEN+SEQ+CMD+DATA 共 (len+3) 字节 */
    buf[4 + len] = crc8(&buf[1], (uint16_t)(3u + len));
    return (uint16_t)(5u + len);
}

/* ================= 字节序打包/解包(小端) ================= */

void comm_pack_i16(uint8_t *p, int16_t v){
    uint16_t u = (uint16_t)v;
    p[0] = (uint8_t)(u & 0xFFu);
    p[1] = (uint8_t)(u >> 8);
}

void comm_pack_i32(uint8_t *p, int32_t v){
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)(u & 0xFFu);
    p[1] = (uint8_t)((u >> 8) & 0xFFu);
    p[2] = (uint8_t)((u >> 16) & 0xFFu);
    p[3] = (uint8_t)((u >> 24) & 0xFFu);
}

int16_t comm_unpack_i16(const uint8_t *p){
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int32_t comm_unpack_i32(const uint8_t *p){
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
