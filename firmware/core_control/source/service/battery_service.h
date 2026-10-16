#ifndef BATTERY_SERVICE_H
#define BATTERY_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* include files */
#include <stdint.h>
#include <stdbool.h>

/* functions declaration */
void     battery_service_init(void);
void     battery_service_poll(bool under_load); /* 周期采样+滤波+估算电量; under_load=带载时冻结低电量判定 */
uint16_t battery_service_get_voltage_mv(void); /* 电池电压(mV) */
uint8_t  battery_service_get_soc_percent(void);/* 剩余电量(0~100%) */
uint16_t battery_service_get_capacity_mah(void);/* 剩余容量(mAh) */
uint8_t  battery_service_is_low_battery(void); /* 是否低电量(1=低, 0=正常) */

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_SERVICE_H */
