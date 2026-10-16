#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor.h"

/* task functions */
void motor_task_init(void);

/* 转速读取接口(由电机任务周期刷新) */
int32_t motor_get_speed_rpm(motor_id_t m_id);   /* 转速, 单位 RPM */
int32_t motor_get_speed_cps(motor_id_t m_id);   /* 转速, 单位 编码器计数/秒 */

#ifdef __cplusplus
}
#endif

#endif  /* MOTOR_TASK_H */
