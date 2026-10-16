/**
 * @file bsp_adc.h
 * @author Davyrefss
 * @brief 
 *
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

/* include files */
#include <stdint.h>
#include "main.h"

/* defines */
#define BAT_ADC_CH           ADC_CHANNEL_14  /* 电池电压检测：PC4 = ADC1_IN14 */
#define BAT_ADC_VREF_V       3.3f            /* ADC 参考电压(V) */
#define BAT_ADC_VDIV_R1_KOHM 10.0f           /* 上分压电阻(kΩ) */
#define BAT_ADC_VDIV_R2_KOHM 3.3f            /* 下分压电阻(kΩ, 接地) */
/* 分压比 = (R1+R2)/R2 ≈ 4.0303, 由分压点电压反算电池电压 */
#define BAT_ADC_VDIV_RATIO   ((BAT_ADC_VDIV_R1_KOHM + BAT_ADC_VDIV_R2_KOHM) / BAT_ADC_VDIV_R2_KOHM)


/* function declarations */
void adc_init(void);
uint32_t adc_get_average(uint8_t times);
float adc_get_battery_voltage(void);
uint16_t adc_get_battery_voltage_mv(void);
float adc_get_measure_voltage(void);

#endif /* BSP_ADC_H */
