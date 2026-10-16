#include "motor_task.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include <stdint.h>

/* const values */
static const uint16_t motor_task_period_ms = 10;   /* 控制周期: 10ms(100Hz) */

/* static variables */
static TaskHandle_t motor_task_handle = NULL;      /* 任务句柄 */
static int32_t motor_speed_cps[MOTOR_NUM] = {0};   /* 各电机转速(编码器计数/秒) */
static int32_t motor_speed_rpm[MOTOR_NUM] = {0};   /* 各电机转速(RPM) */

/**
 * @brief 电机任务: 周期采样编码器, 计算各电机转速
 * @param argument 未使用
 * @note 只在这里调用 motor_get_position(), 其它地方统一读 motor_get_speed_*()
 *       因为 motor_get_position() 内部会累加位置, 多处调用会重复计数
 */
static void motor_task(){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int32_t last_pos[MOTOR_NUM];
    int32_t pos;

    /* 首次读取, 建立位置基准 */
    for (int i = 0; i < MOTOR_NUM; i++) {
        last_pos[i] = motor_get_position((motor_id_t)i);
    }

    for (;;) {
        /* 任务启动后先睡满一个周期才第一次采样，第一次速度就是整整一个 motor_task_period_ms 的增量 */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(motor_task_period_ms));

        for (int i = 0; i < MOTOR_NUM; i++) {
            pos = motor_get_position((motor_id_t)i);

            /* 转速(计数/秒) = 周期内位置增量 / 周期时间 */
            motor_speed_cps[i] = (pos - last_pos[i]) * 1000 / motor_task_period_ms;
            /* 转速(RPM) = 计数/秒 / 每圈计数 * 60 */
            motor_speed_rpm[i] = motor_speed_cps[i] * 60 / motor_get_enc_cpr();

            last_pos[i] = pos;
        }
    }
}

/**
 * @brief 电机任务init
 */
void motor_task_init(void){
    BaseType_t ret = xTaskCreate(motor_task,           /* 任务函数 */
                                "motor_task",          /* 任务名 */
                                128,                   /* 栈深度(字) */
                                NULL,                  /* 任务参数 */
                                tskIDLE_PRIORITY + 2,  /* 优先级: 高于LED任务 */
                                &motor_task_handle);   /* 任务句柄 */
    if (ret != pdPASS) {
        printf("create motor_task failed\r\n");
    }
}

/**
 * @brief 获取电机当前转速(RPM)
 */
int32_t motor_get_speed_rpm(motor_id_t m_id){
    return motor_speed_rpm[m_id];
}

/**
 * @brief 获取电机当前转速(编码器计数/秒)
 */
int32_t motor_get_speed_cps(motor_id_t m_id){
    return motor_speed_cps[m_id];
}
