#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_motor.h"   /* 复用 motor_id_t / MOTOR_NUM */

#ifdef __cplusplus
extern "C" {
#endif

/* 速度环控制参数(可在线调) */
typedef struct {
    float   kp;          /* 比例增益 [duty千分比 / RPM] */
    float   ki;          /* 积分增益 [duty千分比 / (RPM·s)] */
    float   ff_gain;     /* 速度前馈增益 [duty千分比 / RPM] (= 1/K) */
    int16_t ff_offset;   /* 死区前馈偏移 [千分比] (= u0, 稳态死区补偿) */
    int16_t slew_rpm_s;  /* 参考转速爬升率 [rpm/s] */
    int16_t fade_rpm;    /* 起步补偿衰减区间 [rpm] */
    int16_t stiction;    /* 静摩擦额外起步补偿 [千分比] (超出死区部分) */
} motor_ctrl_cfg_t;

void    motor_service_init(void);
void    motor_service_set_speed(motor_id_t id, int16_t rpm);      /* 目标转速 RPM, 闭环 */
void    motor_service_set_all_speed(const int16_t rpm[MOTOR_NUM]);
void    motor_service_set_duty(motor_id_t id, int16_t duty);      /* 开环占空比 -1000~1000, 调试用 */
void    motor_service_brake(motor_id_t id);
void    motor_service_brake_all(void);
void    motor_service_set_ctrl(motor_id_t id, const motor_ctrl_cfg_t *cfg); /* 在线调参 */
void    motor_service_poll(void);                                 /* 周期: M/T测速 + 速度环PID */
int32_t motor_service_get_position(motor_id_t id);
int16_t motor_service_get_speed_rpm(motor_id_t id);
bool    motor_service_is_active(void);                            /* 任一电机被命令或仍在转(带载判断) */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SERVICE_H */
