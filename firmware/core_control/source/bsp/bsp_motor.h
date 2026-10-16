/**
 * @file bsp_motor.h
 * @author Davyrefss
 * @brief motor bsp
 *
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

/* 电机数量 */
#define MOTOR_NUM             4

/* 硬件常量 */
#define BSP_MOTOR_PWM_PERIOD  3600u   /* ARR=3599, 满幅 CCR 计数值 */
#define BSP_MOTOR_ENC_CPR     1320u   /* 输出轴每圈计数(11 PPR × 4 倍频 × 30:1 减速比) */

typedef enum {
    MOTOR1 = 0,
    MOTOR2,
    MOTOR3,
    MOTOR4,
} motor_id_t;

/* 只做硬件：设占空比 / 读编码器，不含方向与位置策略 */
void     bsp_motor_init(motor_id_t id);
void     bsp_motor_all_init(void);
void     bsp_motor_set_duty(motor_id_t id, uint16_t in1_duty, uint16_t in2_duty); /* 0~PERIOD */
uint16_t bsp_motor_get_encoder(motor_id_t id);

#endif /* BSP_MOTOR_H */

