#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* include files */
#include <stdint.h>

/* ============================================================================
 * 与上位机的二进制帧协议(小端字节序)
 * 帧格式:
 *   | SOF(1) | LEN(1) | SEQ(1) | CMD(1) | DATA(LEN) | CRC8(1) |
 *
 *   SOF  : 帧头, 固定 0xA5
 *   LEN  : DATA 载荷字节数(0 ~ COMM_MAX_PAYLOAD)
 *   SEQ  : 帧序号, 每发一帧自增, 应答帧携带原帧 SEQ 用于对应
 *   CMD  : 命令字(见下方定义)
 *   DATA : 载荷, LEN 字节
 *   CRC8 : 对 LEN~DATA 共 (LEN+3) 字节校验; 多项式 0x31, 初值 0xFF, 无反射
 *
 * 上位机发送帧时: LEN = 载荷长度, CRC 覆盖 LEN/SEQ/CMD/DATA;
 * 校验失败或帧超时, MCU 丢弃该帧。
 * ============================================================================ 
 */

/* 常量 */
#define COMM_SOF              0xA5u
#define COMM_MAX_PAYLOAD      32u
#define COMM_MAX_FRAME        (COMM_MAX_PAYLOAD + 5u)    /* SOF+LEN+SEQ+CMD+DATA+CRC */
#define COMM_FRAME_TIMEOUT_MS 20u                        /* 帧超时(ms), 超时丢弃残缺帧 */

/* 下行命令(上位机 -> MCU) */
#define COMM_CMD_PING          0x00u   /* 心跳/探测, 无载荷 */
#define COMM_CMD_SET_SPEED     0x01u   /* 单电机调速: [id:u8][rpm:i16] */
#define COMM_CMD_SET_ALL_SPEED 0x02u   /* 四电机调速: [rpm1..rpm4:i16] */
#define COMM_CMD_BRAKE         0x03u   /* 单电机刹车: [id:u8] */
#define COMM_CMD_BRAKE_ALL     0x04u   /* 全部刹车, 无载荷 */
#define COMM_CMD_SET_DUTY      0x05u   /* 单电机开环占空比: [id:u8][duty:i16] */
#define COMM_CMD_SET_CTRL      0x06u   /* 控制参数: [id:u8][kp:f32][ki:f32][ff:f32][ff_offset:i16][stiction:i16][slew:i16][fade:i16], id=0xFF 广播 */
#define COMM_CMD_GET_TELEMETRY 0x10u   /* 请求一次遥测, 无载荷 */
#define COMM_CMD_GET_BATTERY   0x11u   /* 请求一次电池信息, 无载荷 */

/* 上行命令(MCU -> 上位机) */
#define COMM_CMD_ACK           0x80u   /* 应答: [ack_cmd:u8][result:u8] */
#define COMM_CMD_TELEMETRY     0x81u   /* 遥测: 4x speed_rpm[i16] + 4x position[i32] */
#define COMM_CMD_BATTERY       0x82u   /* 电池: voltage_mv[u16] + soc[u8] + capacity_mah[u16] + flags[u8] */

/* 应答结果码 */
#define COMM_RESULT_OK         0x00u   /* 成功 */
#define COMM_RESULT_ERR_LEN    0x01u   /* 载荷长度错误 */
#define COMM_RESULT_ERR_PARAM  0x02u   /* 参数非法(如电机ID越界) */

/* 遥测载荷长度: 4*2(转速) + 4*4(位置) */
#define COMM_TELEMETRY_LEN     24u

/* 电池载荷长度: 2(电压mV) + 1(电量%) + 2(剩余容量mAh) + 1(标志位) */
#define COMM_BATTERY_LEN       6u

/* 电池标志位 */
#define COMM_BATTERY_FLAG_LOW  0x01u   /* bit0: 低电量 */

/* 解析出的帧结构 */
typedef struct {
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t data[COMM_MAX_PAYLOAD];
} comm_frame_t;

/* 初始化 */
void comm_protocol_init(void);

/* 喂入 1 字节, 解析出完整有效帧返回 1, 否则 0 */
int comm_protocol_feed(uint8_t byte, uint32_t now_tick);

/* 周期调用: 检查帧超时并复位解析状态(now_tick 为 FreeRTOS tick, 1 tick=1ms) */
void comm_protocol_poll(uint32_t now_tick);

/* 最近一次解析出的帧(仅 feed 返回 1 后有效) */
const comm_frame_t *comm_protocol_get_frame(void);

/* 组帧: 输出到 buf(需 >= COMM_MAX_FRAME 字节), 返回帧总长度 */
uint16_t comm_frame_build(uint8_t *buf, uint8_t cmd, uint8_t seq,
                          const uint8_t *data, uint8_t len);

/* 字节序打包/解包(小端) */
void     comm_pack_i16(uint8_t *p, int16_t v);
void     comm_pack_i32(uint8_t *p, int32_t v);
void     comm_pack_f32(uint8_t *p, float v);
int16_t  comm_unpack_i16(const uint8_t *p);
int32_t  comm_unpack_i32(const uint8_t *p);
float    comm_unpack_f32(const uint8_t *p);

#ifdef __cplusplus
}
#endif

#endif  /* COMM_PROTOCOL_H */
