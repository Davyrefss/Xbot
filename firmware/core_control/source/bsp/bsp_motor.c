/**
 * @file bsp_motor.c
 * @author Davyrefss
 * @brief motor bsp
 * *************************************************************************
 * 电机驱动芯片: AM2857 (双通道H桥, IN1/IN2 控制)
 * 真值表:
 *  IN1=H, IN2=L  -> 正转 (IN1 的 PWM 占空比调速)
 *  IN1=L, IN2=H  -> 反转 (IN2 的 PWM 占空比调速)
 *  IN1=L, IN2=L  -> 停止/滑行
 *  IN1=H, IN2=H  -> 刹车(OUT 短接)
 *
 * motor mappimg
 *  M1A=PC6 =TIM8_CH1   M1B=PC7 =TIM8_CH2
 *  M2A=PC8 =TIM8_CH3   M2B=PC9 =TIM8_CH4
 *  M3A=PA11=TIM1_CH4   M3B=PA8 =TIM1_CH1   （普通输出，PartialRemap）
 *  M4A=PB0 =TIM1_CH2N  M4B=PB1 =TIM1_CH3N  （互补输出，PartialRemap）
 *  CubeMX ---> TIM1 TIM8 PSC=0/ARR=3599（20kHz）
 * 
 * encoder mapping
 * M1 -- TIM2(PA15, PB3)
 * M2 -- TIM4(PB6, PB7)
 * M3 -- TIM5(PA0, PA1)
 * M4 -- TIM3(PA6, PA7)
 * 
 * 编码器: 11 PPR(装在电机轴上), Encoder编码器模式 TI1 and TI2 --> (4倍频) -> 电机轴每圈 44 计数
 *         减速比 30:1, 输出轴每圈 = 44 × 30 = 1320 计数(见 bsp_motor.h 的 BSP_MOTOR_ENC_CPR)
 * *************************************************************************
 */

/* include files */
#include <stdint.h>
#include "stm32f1xx_hal.h" 
#include "bsp_motor.h"
#include "tim.h"

/* defines */
#define MOTOR_CH_M1A   TIM_CHANNEL_1
#define MOTOR_CH_M1B   TIM_CHANNEL_2
#define MOTOR_CH_M2A   TIM_CHANNEL_3
#define MOTOR_CH_M2B   TIM_CHANNEL_4
#define MOTOR_CH_M3A   TIM_CHANNEL_4
#define MOTOR_CH_M3B   TIM_CHANNEL_1
#define MOTOR_CH_M4A   TIM_CHANNEL_2
#define MOTOR_CH_M4B   TIM_CHANNEL_3

/* typedefs */
typedef struct {
    TIM_HandleTypeDef *pwm_htim;
    uint32_t           in1_ch;
    uint32_t           in2_ch;
    uint8_t            in1_inv;   /* 1=互补通道(CHxN), 需反算比较值 */
    uint8_t            in2_inv;
    TIM_HandleTypeDef *enc_htim;
} motor_hw_config_t;

/* static variables */
/* 纯硬件映射表, 不可变 */
static const motor_hw_config_t motor_hw[MOTOR_NUM] = {
    [MOTOR1] = { &htim8, MOTOR_CH_M1A, MOTOR_CH_M1B, 0, 0, &htim2 },
    [MOTOR2] = { &htim8, MOTOR_CH_M2A, MOTOR_CH_M2B, 0, 0, &htim4 },
    [MOTOR3] = { &htim1, MOTOR_CH_M3A, MOTOR_CH_M3B, 0, 0, &htim5 },
    [MOTOR4] = { &htim1, MOTOR_CH_M4A, MOTOR_CH_M4B, 1, 1, &htim3 },
};

static void ch_set_duty(const motor_hw_config_t *m, uint32_t ch, uint16_t duty, uint8_t inv)
{
    uint16_t ccr = inv ? (uint16_t)(BSP_MOTOR_PWM_PERIOD - duty) : duty;
    __HAL_TIM_SET_COMPARE(m->pwm_htim, ch, ccr);
}

void bsp_motor_set_duty(motor_id_t id, uint16_t in1_duty, uint16_t in2_duty)
{
    if (id >= MOTOR_NUM) return;
    const motor_hw_config_t *m = &motor_hw[id];
    ch_set_duty(m, m->in1_ch, in1_duty, m->in1_inv);
    ch_set_duty(m, m->in2_ch, in2_duty, m->in2_inv);
}

void bsp_motor_init(motor_id_t id)
{
    const motor_hw_config_t *m = &motor_hw[id];

    HAL_TIM_Encoder_Start(m->enc_htim, TIM_CHANNEL_ALL);

    if (m->in1_inv) HAL_TIMEx_PWMN_Start(m->pwm_htim, m->in1_ch);
    else            HAL_TIM_PWM_Start(m->pwm_htim, m->in1_ch);
    if (m->in2_inv) HAL_TIMEx_PWMN_Start(m->pwm_htim, m->in2_ch);
    else            HAL_TIM_PWM_Start(m->pwm_htim, m->in2_ch);

    /* 高级定时器 TIM1/8 必须使能主输出 MOE */
    if (m->pwm_htim->Instance == TIM1 || m->pwm_htim->Instance == TIM8) {
        __HAL_TIM_MOE_ENABLE(m->pwm_htim);
    }

    bsp_motor_set_duty(id, 0, 0);   /* 初始停止 */
}

void bsp_motor_all_init(void)
{
    for (motor_id_t id = 0; id < MOTOR_NUM; id++) {
        bsp_motor_init(id);
    }
}

uint16_t bsp_motor_get_encoder(motor_id_t id)
{
    return __HAL_TIM_GET_COUNTER(motor_hw[id].enc_htim);
}
