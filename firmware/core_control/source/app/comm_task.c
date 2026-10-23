/**
 * @file comm_task.c
 * @brief 通信任务：App 层，把串口字节喂给协议 → 分发帧 → 回 ACK/遥测
 */

/* include files */
#include "comm_task.h"
#include "comm_protocol.h"
#include "motor_service.h"   /* handle_frame 调它; 同时带来 bsp_motor.h 里的 MOTOR_NUM */
#include "battery_service.h"
#include "system_state.h"
#include "bsp_usart.h"
#include "osal.h"
#include "console.h"
#include <stdio.h>

/* defines */
#define COMM_TASK_PERIOD_MS    10u
#define COMM_TASK_STACK_WORDS  256u
#define COMM_TASK_PRIORITY     OSAL_PRIO_NORMAL   /* 低于 motor_task(控制优先) */

static osal_task_handle_t comm_task_handle = NULL;      /* 通信任务句柄 */

/* functions implementation */
/**
 * @brief 构建并发送一帧通信数据
 * @param cmd 命令码
 * @param seq 序列号
 * @param data 数据指针
 * @param len 数据长度
 */
static void comm_send_frame(uint8_t cmd, uint8_t seq, const uint8_t *data, uint8_t len)
{
    uint8_t buf[COMM_MAX_FRAME];
    uint16_t n = comm_frame_build(buf, cmd, seq, data, len);
    bsp_usart1_send(buf, n);
}

/**
 * @brief 发送ACK响应帧
 * @param frame 接收到的命令帧
 * @param result 结果码
 */
static void comm_send_ack(const comm_frame_t *frame, uint8_t result)
{
    uint8_t data[2] = { frame->cmd, result };   /* ACK 载荷: [原cmd][结果码] */
    comm_send_frame(COMM_CMD_ACK, frame->seq, data, 2);
}

/**
 * @brief 发送遥测帧
 * @param frame 接收到命令帧
 */
static void comm_send_telemetry(const comm_frame_t *frame)
{
    uint8_t data[COMM_TELEMETRY_LEN];

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        comm_pack_i16(&data[2 * i],     motor_service_get_speed_rpm(i));  /* 4×转速 */
        comm_pack_i32(&data[8 + 4 * i], motor_service_get_position(i));   /* 4×位置 */
    }
    comm_send_frame(COMM_CMD_TELEMETRY, frame->seq, data, COMM_TELEMETRY_LEN);
}

/**
 * @brief 发送电池状态帧
 * @param frame 接收到的命令帧
 */
static void comm_send_battery(const comm_frame_t *frame)
{
    uint8_t data[COMM_BATTERY_LEN];

    /* 电压/容量均 < 32768, 用 i16 打包(小端)即可无损携带 u16 值 */
    comm_pack_i16(&data[0], (int16_t)battery_service_get_voltage_mv());
    data[2] = battery_service_get_soc_percent();
    comm_pack_i16(&data[3], (int16_t)battery_service_get_capacity_mah());
    data[5] = battery_service_is_low_battery() ? COMM_BATTERY_FLAG_LOW : 0u;
    comm_send_frame(COMM_CMD_BATTERY, frame->seq, data, COMM_BATTERY_LEN);
}

/**
 * @brief 处理接收到的命令帧
 * @param frame 接收到的命令帧
 */

static void handle_frame(const comm_frame_t *frame)
{
    /* 收到上位机任意有效帧 -> 进入正常运行态 */
    system_state_set(STATE_NORMAL);

    switch (frame->cmd) {

    case COMM_CMD_PING:
        comm_send_ack(frame, COMM_RESULT_OK);
        break;

    case COMM_CMD_SET_SPEED: {
        if (frame->len != 3) { comm_send_ack(frame, COMM_RESULT_ERR_LEN); break; }
        uint8_t id = frame->data[0];
        if (id >= MOTOR_NUM) { comm_send_ack(frame, COMM_RESULT_ERR_PARAM); break; }
        int16_t rpm = comm_unpack_i16(&frame->data[1]);
        motor_service_set_speed(id, rpm);
        comm_send_ack(frame, COMM_RESULT_OK);
        break;
    }

    case COMM_CMD_SET_ALL_SPEED: {
        if (frame->len != 8) { comm_send_ack(frame, COMM_RESULT_ERR_LEN); break; }
        int16_t rpm[MOTOR_NUM];
        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            rpm[i] = comm_unpack_i16(&frame->data[2 * i]);
        }
        motor_service_set_all_speed(rpm);
        comm_send_ack(frame, COMM_RESULT_OK);
        break;
    }

    case COMM_CMD_SET_DUTY: {
        if (frame->len != 3) { comm_send_ack(frame, COMM_RESULT_ERR_LEN); break; }
        uint8_t id = frame->data[0];
        if (id >= MOTOR_NUM) { comm_send_ack(frame, COMM_RESULT_ERR_PARAM); break; }
        int16_t duty = comm_unpack_i16(&frame->data[1]);
        motor_service_set_duty(id, duty);
        comm_send_ack(frame, COMM_RESULT_OK);
        break;
    }

    case COMM_CMD_SET_CTRL: {
        if (frame->len != 21) { comm_send_ack(frame, COMM_RESULT_ERR_LEN); break; }
        uint8_t id = frame->data[0];
        motor_ctrl_cfg_t cfg;
        cfg.kp         = comm_unpack_f32(&frame->data[1]);
        cfg.ki         = comm_unpack_f32(&frame->data[5]);
        cfg.ff_gain    = comm_unpack_f32(&frame->data[9]);
        cfg.ff_offset  = comm_unpack_i16(&frame->data[13]);
        cfg.stiction   = comm_unpack_i16(&frame->data[15]);
        cfg.slew_rpm_s = comm_unpack_i16(&frame->data[17]);
        cfg.fade_rpm   = comm_unpack_i16(&frame->data[19]);
        if (id == 0xFFu) {                          /* 广播: 所有电机同参 */
            for (uint8_t i = 0; i < MOTOR_NUM; i++) motor_service_set_ctrl(i, &cfg);
        } else if (id < MOTOR_NUM) {
            motor_service_set_ctrl(id, &cfg);
        } else {
            comm_send_ack(frame, COMM_RESULT_ERR_PARAM); break;
        }
        comm_send_ack(frame, COMM_RESULT_OK);
        break;
    }

    case COMM_CMD_BRAKE: {
        if (frame->len != 1) { comm_send_ack(frame, COMM_RESULT_ERR_LEN); break; }
        uint8_t id = frame->data[0];
        if (id >= MOTOR_NUM) { comm_send_ack(frame, COMM_RESULT_ERR_PARAM); break; }
        motor_service_brake(id);
        comm_send_ack(frame, COMM_RESULT_OK);
        break;
    }

    case COMM_CMD_BRAKE_ALL:
        motor_service_brake_all();
        comm_send_ack(frame, COMM_RESULT_OK);
        break;

    case COMM_CMD_GET_TELEMETRY:
        comm_send_telemetry(frame);   /* 遥测请求回 TELEMETRY 帧，不回 ACK */
        break;

    case COMM_CMD_GET_BATTERY:
        comm_send_battery(frame);     /* 电池信息请求回 BATTERY 帧，不回 ACK */
        break;

    default:
        comm_send_ack(frame, COMM_RESULT_ERR_PARAM);   /* 未知命令 */
        break;
    }
}

/* ============ 任务 ============ */

static void comm_task(void *arg)
{
    (void)arg;
    uint8_t byte;
    osal_wake_time_t wake;

    osal_wake_time_init(&wake);
    comm_protocol_init();

    for (;;) {
#ifndef ORDINARY_DEBUG        
        /* 1. 把缓冲里的字节全部喂给协议，出完整帧就分发 */
        while (bsp_usart1_read(&byte)) {
            if (comm_protocol_feed(byte, osal_time_get_ms())) {
                handle_frame(comm_protocol_get_frame());
            }
        }

        /* 2. 帧超时复位（丢弃残缺帧） */
        comm_protocol_poll(osal_time_get_ms());
#endif
        /* 3. 固定周期调度 */
        osal_delay_until(&wake, COMM_TASK_PERIOD_MS);
    }
}
 
void comm_task_init(void) 
{
    bsp_usart1_init();
    if (!osal_task_create("comm_task", COMM_TASK_STACK_WORDS, COMM_TASK_PRIORITY,
                          comm_task, NULL, &comm_task_handle)) {
        printf("create comm_task failed\r\n");
    }
}
