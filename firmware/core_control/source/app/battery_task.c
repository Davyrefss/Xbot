/**
 * @file battery_task.c
 * @brief 电池任务：App 层，周期采样电池电压并估算电量/剩余容量
 */

/* include files */
#include "battery_task.h"
#include "battery_service.h"
#include "motor_service.h"
#include "osal.h"
#include <stdio.h>

/* defines */
#define BATTERY_TASK_PERIOD_MS    100u
#define BATTERY_TASK_STACK_WORDS  128u
#define BATTERY_TASK_PRIORITY     OSAL_PRIO_LOW

/* static variables */
static osal_task_handle_t battery_task_handle = NULL;

/* functions implementation */
static void battery_task(void *arg)
{
    (void)arg;
    osal_wake_time_t wake;

    osal_wake_time_init(&wake);
    battery_service_init();

    for (;;) {
        battery_service_poll(motor_service_is_active()); /* 采样+滤波+电量估算; 带载时冻结低电量判定 */
        osal_delay_until(&wake, BATTERY_TASK_PERIOD_MS);
    }
}

void battery_task_init(void)
{
    if (!osal_task_create("battery_task", BATTERY_TASK_STACK_WORDS, BATTERY_TASK_PRIORITY,
                          battery_task, NULL, &battery_task_handle)) {
        printf("create battery_task failed\r\n");
    }
}
