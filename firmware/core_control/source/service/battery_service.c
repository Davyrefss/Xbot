/**
 * @file battery_service.c
 * @brief 电池业务层: 采样电池分压电压 -> 一阶低通滤波 -> 电压法估算电量/剩余容量
 *
 * 硬件: 电池正极经 10k(上)/3.3k(下) 电阻分压接地, 分压点到 ADC1_IN14(PC4);
 *       bsp_adc 负责把分压点电压反算成电池电压(mV)。
 *
 * 电量估算: 3S 锂离子电池(标称 11.1V, 满 12.6V, 截止约 9.0V)的开路电压-电量曲线
 *       分段线性插值。注意电压法在中段(30%~80%)平台较平、精度有限, 建议按实际电池
 *       实测标定 soc_curve[]; 若换用 LiFePO4 / 铅酸电池, 需重排该表。
 */

/* include files */
#include "battery_service.h"
#include "bsp_adc.h"

/* defines */
#define BATTERY_FULL_CAPACITY_MAH   2550u   /* 电池标称容量(mAh), 按电池丝印填写 */
#define BATTERY_VOLTAGE_LPF_ALPHA   0.1f    /* 电压一阶低通系数(0~1), 越小越平滑/滞后越大 */
#define BATTERY_LOW_SOC_PERCENT     20u     /* 电量低于此值判低电量 */
#define BATTERY_LOW_RECOVER_PERCENT 25u     /* 电量回升到此值解除(滞回, 防边界抖动) */
#define BATTERY_LOW_DEBOUNCE_CNT    5u      /* 连续 N 次采样都低才置位(100ms周期->约500ms消抖) */

/* 3S 锂离子电池 开路电压(mV) -> 电量(%) 对照表, 电压从高到低排列(分段线性插值用) */
static const struct {
    uint16_t voltage_mv;
    uint8_t  soc;      /* 0~100% */
} soc_curve[] = {
    { 12600, 100 },
    { 12180,  90 },
    { 11940,  80 },
    { 11760,  70 },
    { 11610,  60 },
    { 11460,  50 },
    { 11310,  40 },
    { 11160,  30 },
    { 10980,  20 },
    { 10800,  10 },
    { 10500,   5 },
    {  9000,   0 },
};

/* static variables */
static uint16_t voltage_mv;      /* 滤波后的电池电压(mV) */
static uint8_t  soc_percent;     /* 剩余电量 0~100% */
static volatile uint8_t low_battery;   /* 低电量标志(0=正常 1=低), 跨任务共享 */
static uint8_t low_count;              /* 连续低电量采样计数(消抖) */

/* functions implementation */

/**
 * @brief 由电压(mV)分段线性插值估算电量(%)
 */
static uint8_t soc_from_voltage_mv(uint16_t mv)
{
    uint8_t n = (uint8_t)(sizeof(soc_curve) / sizeof(soc_curve[0]));

    if (mv >= soc_curve[0].voltage_mv)     return 100;   /* 高于最高点 */
    if (mv <= soc_curve[n - 1].voltage_mv) return 0;     /* 低于截止点 */

    for (uint8_t i = 0; i < n - 1; i++) {
        uint16_t v_hi = soc_curve[i].voltage_mv;      /* 高压点(高电量) */
        uint16_t v_lo = soc_curve[i + 1].voltage_mv;  /* 低压点(低电量) */
        if (mv <= v_hi && mv >= v_lo) {
            /* soc = soc_lo + (soc_hi - soc_lo) * (mv - v_lo) / (v_hi - v_lo) */
            int32_t soc = (int32_t)soc_curve[i + 1].soc +
                          (int32_t)(soc_curve[i].soc - soc_curve[i + 1].soc) *
                          (int32_t)(mv - v_lo) / (int32_t)(v_hi - v_lo);
            return (uint8_t)soc;
        }
    }
    return 0;
}

void battery_service_init(void)
{
    adc_init();
    voltage_mv  = 0;
    soc_percent = 0;
    low_battery = 0;
    low_count   = 0;
}

void battery_service_poll(bool under_load)
{
    uint16_t raw = adc_get_battery_voltage_mv();

    /* 一阶低通: 平滑采样抖动, 电池电压本身变化很慢 */
    if (voltage_mv == 0) {
        voltage_mv = raw;   /* 首帧直接采入, 避免从 0 慢慢爬 */
    } else {
        voltage_mv = (uint16_t)((float)voltage_mv +
                                BATTERY_VOLTAGE_LPF_ALPHA * ((float)raw - (float)voltage_mv));
    }
    soc_percent = soc_from_voltage_mv(voltage_mv);

    /* 带载(电机运行)时电压被拉低、SoC 读数不可靠, 冻结判定保持上次结果 */
    if (under_load) {
        return;
    }

    /* 低电量判定: 连续消抖 + 滞回, 避免瞬时跌落/边界抖动误报 */
    if (soc_percent < BATTERY_LOW_SOC_PERCENT) {
        if (low_count < BATTERY_LOW_DEBOUNCE_CNT) {
            low_count++;
        }
        if (low_count >= BATTERY_LOW_DEBOUNCE_CNT) {
            low_battery = 1;
        }
    } else if (soc_percent > BATTERY_LOW_RECOVER_PERCENT) {
        low_count   = 0;
        low_battery = 0;
    }
    /* 介于 20%~25% 滞回区: 保持当前状态 */
}

uint16_t battery_service_get_voltage_mv(void)
{
    return voltage_mv;
}

uint8_t battery_service_get_soc_percent(void)
{
    return soc_percent;
}

uint16_t battery_service_get_capacity_mah(void)
{
    return (uint16_t)((uint32_t)soc_percent * BATTERY_FULL_CAPACITY_MAH / 100u);
}

uint8_t battery_service_is_low_battery(void)
{
    return low_battery;
}
