/**
 * @file bsp_adc.c
 * @author Davyrefss
 * @brief  -采样时间改为 239.5 周期,而电池分压电阻较大，采样太短会让读数明显偏低/跳动
 *
 */

/* include files */
#include <stdint.h>
#include "bsp_adc.h"
#include "adc.h"

/* static variables */
static ADC_ChannelConfTypeDef adc_channel_config = {
    .Channel = BAT_ADC_CH,
    .Rank = ADC_REGULAR_RANK_1,
    .SamplingTime = ADC_SAMPLETIME_239CYCLES_5,
};

/* functions implementation */

/**
 * @brief 获取adc测量电压值
 *
 */
static uint16_t adc_get_value(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return adc_value;
}

/**
 * @brief adc 初始化
 *
 */
void adc_init(void) {
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_ConfigChannel(&hadc1, &adc_channel_config);
}

/**
 * @brief 获取adc平均值
 * @param adc_channel adc通道
 * @param times 采样次数
 * @return uint32_t 平均值
 */
uint32_t adc_get_average(uint8_t times) {
    if (times == 0) {
        return 0;
    }
    uint32_t adc_value_sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        adc_value_sum += adc_get_value();
    }
    return adc_value_sum / times;
}

/**
 * @brief 获取adc测量值
 * 
 */
float adc_get_measure_voltage(void) {
    return (float)adc_get_average(10) * BAT_ADC_VREF_V / 4096.0f;
}

/**
 * @brief 获取电池电压值(V)
 */
float adc_get_battery_voltage(void) {
    return adc_get_measure_voltage() * BAT_ADC_VDIV_RATIO;
}

/**
 * @brief 获取电池电压值(mV)
 */
uint16_t adc_get_battery_voltage_mv(void) {
    return (uint16_t)(adc_get_battery_voltage() * 1000.0f);
}
