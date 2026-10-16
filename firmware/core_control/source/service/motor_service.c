/**
 * @file motor_service.c
 * @brief 电机业务层: 位置累计 + M/T 法测速 + 增量式速度环 PI
 *
 * 控制结构(有刷直流 + 正交编码器, 无电流采样):
 *   目标转速(RPM) -> 参考斜坡 + 速度前馈 + 静摩擦补偿 + 增量式 PI -> 占空比(-1000~1000) -> bsp_motor
 *   编码器 -> M/T 自适应窗口测速 -> 反馈转速
 *
 * 测速说明: 编码器在电机轴(11 PPR×4=44 CPR, 减速比 30:1), 输出轴每圈 1320 计数,
 *   固定窗口"数脉冲"在低速时分辨率极差(20ms 内输出轴 1 RPM 仅 ~0.44 计数),
 *   故采用自适应窗口 M/T 法:
 *   同一窗口同时累计 计数 M 与 精确时间 T(µs), 转速 = M/T;
 *   窗口在 |M|>=MIN_CNT 或 T>=MAX_WINDOW_US 时关闭, 高低速都保分辨率。
 *   测得转速再做一阶低通(系数 MOTOR_SPEED_LPF_ALPHA), 平滑低速量化抖动与测量毛刺。
 */

/* include files */
#include <stdbool.h>
#include "motor_service.h"
#include "bsp_motor.h"
#include "stm32f1xx_hal.h"   /* DWT->CYCCNT 微秒时间基准 */

/* defines */
#define MOTOR_DUTY_MAX            1000     /* 占空比千分比满幅 */
#define MOTOR_SPEED_MIN_CNT       8        /* 至少累计 8 个计数才算一次速度 */
#define MOTOR_SPEED_MAX_WINDOW_US 50000u   /* 低速最多等 50ms 强制出结果 */
#define MOTOR_CTRL_DT_MAX_S       0.05f    /* 控制步长上限保护(秒) */
#define MOTOR_STOP_DEADBAND_RPM   5.0f     /* 目标为0且|实测|<此值 -> 输出清零 */
#define MOTOR_ACTIVE_SPEED_RPM    5.0f     /* |实测|高于此视为电机仍在转(带载判断) */
#define MOTOR_SPEED_LPF_ALPHA     0.35f    /* 测速一阶低通系数(0~1), 越小越平滑/滞后越大, 1=不滤波 */
/* 控制参数缺省值(上电默认, 之后可用 SET_CTRL 命令在线覆盖)
 * 由 autotune.py 对 MD520Z30_12V 整定: 目标 126rpm, 过冲 10%
 * (CPR=1320 减速比 30:1, 测速已加一阶低通 MOTOR_SPEED_LPF_ALPHA 平滑) */
#define MOTOR_DEFAULT_KP          1.974f
#define MOTOR_DEFAULT_KI          10.344f
#define MOTOR_DEFAULT_FF_GAIN     1.645f
#define MOTOR_DEFAULT_FF_OFFSET   436
#define MOTOR_DEFAULT_STICTION    87
#define MOTOR_DEFAULT_SLEW_RPM_S  250
#define MOTOR_DEFAULT_FADE_RPM    50

extern uint32_t SystemCoreClock;

typedef enum {
    MOTOR_MODE_OPENLOOP = 0,   /* 开环: 直接给占空比 */
    MOTOR_MODE_SPEED,          /* 闭环: 目标转速, PID 输出占空比 */
} motor_mode_t;

/* static variables */
static uint16_t enc_last[MOTOR_NUM];      /* 上次编码器原始值(16bit) */
static int32_t  pos_accum[MOTOR_NUM];     /* 累计位置(计数, 带符号) */

static int32_t  m_cnt[MOTOR_NUM];         /* M/T 窗口累计计数 */
static uint32_t m_us[MOTOR_NUM];          /* M/T 窗口累计 µs */
static uint32_t m_last_us[MOTOR_NUM];     /* 上次测速时间戳 µs */
static float    meas_rpm[MOTOR_NUM];      /* M/T 测得转速 */

static motor_mode_t    mode[MOTOR_NUM];
static int16_t         target_rpm[MOTOR_NUM];
static float           ref_rpm[MOTOR_NUM];      /* 斜坡后的参考转速(RPM) */
static motor_ctrl_cfg_t ctrl_cfg[MOTOR_NUM];
static float           pid_out[MOTOR_NUM];      /* -1000..1000 */
static float           pid_err_prev[MOTOR_NUM];

static uint32_t ctrl_last_us;
static uint32_t us_scale = 1;             /* 防止未初始化时除零 */
static bool     us_ready = false;
static bool     inited   = false;

/* ---------- µs 时间基准(DWT 周期计数器, 72MHz/32bit, ~59.6s 翻转) ---------- */
static void micros_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    us_scale = SystemCoreClock / 1000000u;
    us_ready = true;
}

static uint32_t micros(void)
{
    if (!us_ready) micros_init();
    return DWT->CYCCNT / us_scale;
}

/* 开环: 按符号映射占空比到 IN1/IN2 */
static void set_duty(motor_id_t id, int16_t duty)
{
    uint16_t d, in1, in2;

    if (duty > MOTOR_DUTY_MAX)  duty = MOTOR_DUTY_MAX;
    if (duty < -MOTOR_DUTY_MAX) duty = -MOTOR_DUTY_MAX;

    if (duty > 0) {          /* 正转: IN1 调占空比, IN2 拉低 */
        d = (uint16_t)((uint32_t)duty * BSP_MOTOR_PWM_PERIOD / MOTOR_DUTY_MAX);
        in1 = d; in2 = 0;
    } else if (duty < 0) {   /* 反转: IN2 调占空比, IN1 拉低 */
        d = (uint16_t)((uint32_t)(-duty) * BSP_MOTOR_PWM_PERIOD / MOTOR_DUTY_MAX);
        in1 = 0; in2 = d;
    } else {                 /* 停止/滑行 */
        in1 = 0; in2 = 0;
    }
    bsp_motor_set_duty(id, in1, in2);
}

/* 进入闭环前复位 PID/测速窗口, 避免旧积分造成冲击 */
static void reset_control(motor_id_t id)
{
    pid_out[id]      = 0.0f;
    pid_err_prev[id] = 0.0f;
    m_cnt[id]        = 0;
    m_us[id]         = 0;
    m_last_us[id]    = micros();
    ref_rpm[id]      = 0.0f;
}

void motor_service_init(void)
{
    bsp_motor_all_init();
    micros_init();

    for (motor_id_t i = 0; i < MOTOR_NUM; i++) {
        enc_last[i]  = bsp_motor_get_encoder(i);
        pos_accum[i] = 0;
        meas_rpm[i]  = 0.0f;
        mode[i]      = MOTOR_MODE_OPENLOOP;
        target_rpm[i] = 0;
        ctrl_cfg[i].kp         = MOTOR_DEFAULT_KP;
        ctrl_cfg[i].ki         = MOTOR_DEFAULT_KI;
        ctrl_cfg[i].ff_gain    = MOTOR_DEFAULT_FF_GAIN;
        ctrl_cfg[i].ff_offset  = MOTOR_DEFAULT_FF_OFFSET;
        ctrl_cfg[i].stiction   = MOTOR_DEFAULT_STICTION;
        ctrl_cfg[i].slew_rpm_s = MOTOR_DEFAULT_SLEW_RPM_S;
        ctrl_cfg[i].fade_rpm   = MOTOR_DEFAULT_FADE_RPM;
        reset_control(i);
    }
    ctrl_last_us = micros();
    inited = true;
}

void motor_service_set_speed(motor_id_t id, int16_t rpm)
{
    if (id >= MOTOR_NUM) return;
    if (mode[id] != MOTOR_MODE_SPEED) {   /* 开环 -> 闭环 切换时复位积分 */
        reset_control(id);
        mode[id] = MOTOR_MODE_SPEED;
    }
    target_rpm[id] = rpm;
}

void motor_service_set_all_speed(const int16_t rpm[MOTOR_NUM])
{
    for (motor_id_t i = 0; i < MOTOR_NUM; i++) {
        motor_service_set_speed(i, rpm[i]);
    }
}

void motor_service_set_duty(motor_id_t id, int16_t duty)
{
    if (id >= MOTOR_NUM) return;
    mode[id] = MOTOR_MODE_OPENLOOP;
    target_rpm[id] = 0;   /* 开环命令覆盖速度目标, 避免 is_active() 误判带载而冻结低电量 */
    set_duty(id, duty);
}

void motor_service_brake(motor_id_t id)
{
    if (id >= MOTOR_NUM) return;
    mode[id] = MOTOR_MODE_OPENLOOP;
    target_rpm[id] = 0;   /* 清速度目标, 否则 is_active() 永久判带载、冻结低电量 */
    bsp_motor_set_duty(id, BSP_MOTOR_PWM_PERIOD, BSP_MOTOR_PWM_PERIOD);  /* IN1=IN2=H 刹车 */
}

void motor_service_brake_all(void)
{
    for (motor_id_t i = 0; i < MOTOR_NUM; i++) {
        motor_service_brake(i);
    }
}

void motor_service_set_ctrl(motor_id_t id, const motor_ctrl_cfg_t *cfg)
{
    if (id >= MOTOR_NUM || cfg == NULL) return;
    ctrl_cfg[id] = *cfg;
}

void motor_service_poll(void)
{
    uint32_t now_us;
    float dt;

    if (!inited) return;
    now_us = micros();

    /* 控制步长(秒), 首帧/长时间停滞后限幅保护 */
    dt = (float)(now_us - ctrl_last_us) * 1e-6f;
    ctrl_last_us = now_us;
    if (dt <= 0.0f)             dt = 0.01f;
    if (dt > MOTOR_CTRL_DT_MAX_S) dt = MOTOR_CTRL_DT_MAX_S;

    for (motor_id_t i = 0; i < MOTOR_NUM; i++) {
        uint16_t now   = bsp_motor_get_encoder(i);
        int16_t  delta = (int16_t)(now - enc_last[i]);   /* 16bit 差, 自动处理翻转 */
        enc_last[i] = now;
        pos_accum[i] += delta;

        /* --- M/T 自适应窗口测速 --- */
        uint32_t dt_us = now_us - m_last_us[i];
        m_last_us[i] = now_us;
        m_cnt[i] += delta;
        m_us[i]  += dt_us;

        if (m_us[i] >= MOTOR_SPEED_MAX_WINDOW_US ||
            m_cnt[i] >= MOTOR_SPEED_MIN_CNT ||
            m_cnt[i] <= -MOTOR_SPEED_MIN_CNT) {
            if (m_us[i] > 0) {
                /* rpm = 计数 × 60e6 / (每圈计数 × 时间µs) */
                float raw = (float)m_cnt[i] * 60000000.0f /
                            (float)((uint32_t)BSP_MOTOR_ENC_CPR * m_us[i]);
                /* 一阶低通: 平滑低速量化抖动, 输出曲线更平滑(系数越小越平滑但滞后越大) */
                meas_rpm[i] += MOTOR_SPEED_LPF_ALPHA * (raw - meas_rpm[i]);
            }
            m_cnt[i] = 0;
            m_us[i]  = 0;
        }

        /* --- 参考斜坡 + 速度前馈 + 静摩擦补偿 + 增量式速度环 PI --- */
        if (mode[i] == MOTOR_MODE_SPEED) {
            float out;

            /* 参考转速按固定斜率逼近目标, 消除阶跃起步冲击 */
            float step = (float)ctrl_cfg[i].slew_rpm_s * dt;
            float diff = (float)target_rpm[i] - ref_rpm[i];
            if (step <= 0.0f)          ref_rpm[i] = (float)target_rpm[i];  /* 斜率<=0 关闭斜坡 */
            else if (diff > step)      ref_rpm[i] += step;
            else if (diff < -step)     ref_rpm[i] -= step;
            else                       ref_rpm[i] = (float)target_rpm[i];

            if (target_rpm[i] == 0 &&
                meas_rpm[i] > -MOTOR_STOP_DEADBAND_RPM &&
                meas_rpm[i] <  MOTOR_STOP_DEADBAND_RPM) {
                /* 目标 0 且已基本停住: 输出清零, 避免原地抖动 */
                out = 0.0f;
                pid_err_prev[i] = 0.0f;
                pid_out[i] = 0.0f;
                ref_rpm[i] = 0.0f;
            } else {
                float err = ref_rpm[i] - meas_rpm[i];
                float du  = ctrl_cfg[i].kp * (err - pid_err_prev[i]) +
                            ctrl_cfg[i].ki * err * dt;
                pid_err_prev[i] = err;
                pid_out[i] += du;                         /* PI 只负责修正误差 */
                if (pid_out[i] > MOTOR_DUTY_MAX)      pid_out[i] = MOTOR_DUTY_MAX;
                else if (pid_out[i] < -MOTOR_DUTY_MAX) pid_out[i] = -MOTOR_DUTY_MAX;

                /* 速度前馈: 死区偏移 + 参考转速对应的基准占空比 */
                float ff = ctrl_cfg[i].ff_gain * ref_rpm[i] +
                           (float)ctrl_cfg[i].ff_offset * ((target_rpm[i] > 0) - (target_rpm[i] < 0));

                /* 静摩擦额外补偿(超出死区部分): 命令非0时按方向给起步力, 随转速线性衰减到0 */
                float kick = 0.0f;
                if (target_rpm[i] != 0) {
                    float spd  = meas_rpm[i] > 0.0f ? meas_rpm[i] : -meas_rpm[i];
                    float fr   = (float)ctrl_cfg[i].fade_rpm;
                    float fade = (fr > 0.0f) ? (1.0f - spd / fr) : 0.0f;
                    if (fade < 0.0f) fade = 0.0f;
                    float st = (float)ctrl_cfg[i].stiction;
                    kick = (target_rpm[i] > 0) ? st * fade : -st * fade;
                }

                out = ff + kick + pid_out[i];
            }

            if (out > MOTOR_DUTY_MAX)       out = MOTOR_DUTY_MAX;
            else if (out < -MOTOR_DUTY_MAX) out = -MOTOR_DUTY_MAX;

            set_duty(i, (int16_t)out);
        }
    }
}

int32_t motor_service_get_position(motor_id_t id)
{
    if (id >= MOTOR_NUM) return 0;
    return pos_accum[id];
}

int16_t motor_service_get_speed_rpm(motor_id_t id)
{
    if (id >= MOTOR_NUM) return 0;
    return (int16_t)meas_rpm[id];
}

bool motor_service_is_active(void)
{
    for (motor_id_t i = 0; i < MOTOR_NUM; i++) {
        if (target_rpm[i] != 0) return true;                       /* 有目标转速命令 */
        if (meas_rpm[i] > MOTOR_ACTIVE_SPEED_RPM ||
            meas_rpm[i] < -MOTOR_ACTIVE_SPEED_RPM) return true;    /* 开环/惯性仍在转 */
    }
    return false;
}
