/**
 * @file vofa_debug.c
 * @brief VOFA+ 波形调试输出(伏特加 JustFloat 协议)
 *
 * 与上位机二进制协议(comm_protocol)共用 USART1, 二者不能同时发送,
 * 由 comm_task.c 顶部的 VOFA_DEBUG 宏二选一。
 *
 * JustFloat 帧格式:
 *   每通道 4 字节小端 float, 帧尾固定 00 00 80 7F(小端 float = +Inf,
 *   VOFA+ 用它判定帧边界)。本模块固定发送 4 通道: M1..M4 转速(RPM)。
 */

#include "vofa_debug.h"
#include "bsp_usart.h"
#include "motor.h"
#include "motor_task.h"
#include <string.h>

/* 通道数: 4x 转速 RPM */
#define VOFA_CH_NUM        4u
/* 帧尾长度(字节) */
#define VOFA_TAIL_SIZE     4u

/* 发送缓冲: 通道数据 + 帧尾 */
static uint8_t vofa_buf[VOFA_CH_NUM * 4u + VOFA_TAIL_SIZE];

/**
 * @brief 将 float 按小端写入字节流
 */
static void vofa_pack_float(uint8_t *dst, float f){
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    dst[0] = (uint8_t)(bits);
    dst[1] = (uint8_t)(bits >> 8);
    dst[2] = (uint8_t)(bits >> 16);
    dst[3] = (uint8_t)(bits >> 24);
}

/**
 * @brief 发送一帧 JustFloat 波形数据
 * @note  内容: M1..M4 转速(RPM)。
 *        如需加位置曲线, 再补一段循环 pack motor_get_position_ro,
 *        并把 VOFA_CH_NUM 相应增大。
 */
void vofa_debug_send(void){
    uint8_t *p = vofa_buf;

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        vofa_pack_float(p, (float)motor_get_speed_rpm((motor_id_t)i));
        p += 4;
    }

    /* 帧尾: 00 00 80 7F */
    p[0] = 0x00;
    p[1] = 0x00;
    p[2] = 0x80;
    p[3] = 0x7F;

    bsp_usart1_send(vofa_buf, sizeof(vofa_buf));
}
