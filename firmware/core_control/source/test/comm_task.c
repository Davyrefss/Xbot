/**
 * @file comm_task.c
 * @brief USART1 通信任务: 收帧 -> 执行电机指令 -> 周期回传遥测
 *
 * 与上位机通过二进制帧协议通信(见 comm_protocol.h)。
 * 任务周期 5ms 轮询接收缓冲, 解析出完整帧后立即执行;
 * 每 50ms 周期上报一次遥测(各电机转速 RPM + 累计位置)。
 */

/* include files */
#include "comm_task.h"
#include "comm_protocol.h"
#include "bsp_usart.h"
#include "motor.h"
#include "motor_task.h"
#include "vofa_debug.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdint.h>

/* ================= 调试输出开关 =================
 * 定义 VOFA_DEBUG 后:
 *   周期遥测改用 VOFA+ JustFloat 波形(供曲线调试),
 *   二进制遥测与 ACK 应答关闭(避免与波形流混帧)。
 * 注释掉该宏则恢复二进制协议遥测(上位机通信模式)。
 */
// #define VOFA_DEBUG

/* const values */
static const uint16_t comm_task_period_ms = 5;    /* 任务周期: 5ms */
#ifdef VOFA_DEBUG
static const uint16_t telemetry_period_ms = 10;   /* 波形调试周期: 10ms(100Hz) */
#else
static const uint16_t telemetry_period_ms = 50;   /* 遥测周期: 50ms(20Hz) */
#endif

/* static variables */
static TaskHandle_t comm_task_handle = NULL;
#ifndef VOFA_DEBUG
static uint8_t telemetry_seq = 0;                 /* 遥测帧序号 */
#endif

/* static functions */
static void comm_handle_frame(const comm_frame_t *frame);
static void comm_send_ack(uint8_t ack_cmd, uint8_t result, uint8_t seq);
static void comm_send_telemetry(void);

/**
 * @brief 通信任务
 */
static void comm_task(void *argument){
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_telemetry = last_wake;
    uint8_t byte;
    uint32_t now;

    comm_protocol_init();

    for (;;) {
        now = xTaskGetTickCount();
        /* 1. 处理收到的字节 */
        while (bsp_usart1_read(&byte)) {
            if (comm_protocol_feed(byte, now)) {
                comm_handle_frame(comm_protocol_get_frame());
            }
        }

        /* 2. 帧超时复位 */
        comm_protocol_poll(now);

        /* 3. 周期上报遥测 */
        if ((now - last_telemetry) >= pdMS_TO_TICKS(telemetry_period_ms)) {
#ifdef VOFA_DEBUG
            vofa_debug_send();
#else
            comm_send_telemetry();
#endif
            last_telemetry = now;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(comm_task_period_ms));
    }
}

/**
 * @brief 处理一帧命令
 */
static void comm_handle_frame(const comm_frame_t *frame){
    switch (frame->cmd) {
        case COMM_CMD_PING:
            comm_send_ack(frame->cmd, COMM_RESULT_OK, frame->seq);
            break;

        case COMM_CMD_SET_SPEED:
            if (frame->len != 3) {
                comm_send_ack(frame->cmd, COMM_RESULT_ERR_LEN, frame->seq);
                break;
            }
            {
                uint8_t m_id = frame->data[0];
                int16_t speed = comm_unpack_i16(&frame->data[1]);
                if (m_id >= MOTOR_NUM) {
                    comm_send_ack(frame->cmd, COMM_RESULT_ERR_PARAM, frame->seq);
                    break;
                }
                motor_set_speed((motor_id_t)m_id, speed);
                comm_send_ack(frame->cmd, COMM_RESULT_OK, frame->seq);
            }
            break;

        case COMM_CMD_SET_ALL_SPEED:
            if (frame->len != 8) {
                comm_send_ack(frame->cmd, COMM_RESULT_ERR_LEN, frame->seq);
                break;
            }
            for (uint8_t i = 0; i < MOTOR_NUM; i++) {
                motor_set_speed((motor_id_t)i, comm_unpack_i16(&frame->data[2 * i]));
            }
            comm_send_ack(frame->cmd, COMM_RESULT_OK, frame->seq);
            break;

        case COMM_CMD_BRAKE:
            if (frame->len != 1) {
                comm_send_ack(frame->cmd, COMM_RESULT_ERR_LEN, frame->seq);
                break;
            }
            {
                uint8_t m_id = frame->data[0];
                if (m_id >= MOTOR_NUM) {
                    comm_send_ack(frame->cmd, COMM_RESULT_ERR_PARAM, frame->seq);
                    break;
                }
                motor_brake((motor_id_t)m_id);
                comm_send_ack(frame->cmd, COMM_RESULT_OK, frame->seq);
            }
            break;

        case COMM_CMD_BRAKE_ALL:
            for (uint8_t i = 0; i < MOTOR_NUM; i++) {
                motor_brake((motor_id_t)i);
            }
            comm_send_ack(frame->cmd, COMM_RESULT_OK, frame->seq);
            break;

        case COMM_CMD_GET_TELEMETRY:
            comm_send_telemetry();
            break;

        default:
            /* 未知命令, 不应答 */
            break;
    }
}

/**
 * @brief 发送应答帧
 * @param ack_cmd 被应答的命令字
 * @param result 结果码
 * @param seq 原帧序号
 */
static void comm_send_ack(uint8_t ack_cmd, uint8_t result, uint8_t seq){
#ifndef VOFA_DEBUG
    uint8_t data[2] = { ack_cmd, result };
    uint8_t buf[COMM_MAX_FRAME];

    uint16_t len = comm_frame_build(buf, COMM_CMD_ACK, seq, data, 2);
    bsp_usart1_send(buf, len);
#else
    /* 调试模式: 关闭 ACK, 避免二进制帧混入 VOFA+ 波形流 */
    (void)ack_cmd;
    (void)result;
    (void)seq;
#endif
}

/**
 * @brief 发送遥测帧
 * @note  载荷: 4x 转速(RPM, i16) + 4x 累计位置(编码器计数, i32)
 */
static void comm_send_telemetry(void){
#ifndef VOFA_DEBUG
    uint8_t data[COMM_TELEMETRY_LEN];
    uint8_t buf[COMM_MAX_FRAME];

    /* 4x 转速(RPM, i16) */
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        comm_pack_i16(&data[2 * i], (int16_t)motor_get_speed_rpm((motor_id_t)i));
    }
    /* 4x 累计位置(编码器计数, i32) */
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        comm_pack_i32(&data[8 + 4 * i], motor_get_position_ro((motor_id_t)i));
    }

    uint16_t len = comm_frame_build(buf, COMM_CMD_TELEMETRY, telemetry_seq++, data, COMM_TELEMETRY_LEN);
    bsp_usart1_send(buf, len);
#endif
}

/**
 * @brief 通信任务 init
 * @note  启动 USART1 中断接收后创建任务
 */
void comm_task_init(void){
    bsp_usart1_init();   /* 启动 USART1 中断接收 */

    BaseType_t ret = xTaskCreate(comm_task,              /* 任务函数 */
                                 "comm_task",            /* 任务名 */
                                 256,                    /* 栈深度(字) */
                                 NULL,                   /* 任务参数 */
                                 tskIDLE_PRIORITY + 3,   /* 优先级: 高于电机/LED任务 */
                                 &comm_task_handle);     /* 任务句柄 */
    if (ret != pdPASS) {
        printf("create comm_task failed\r\n");
    }
}
