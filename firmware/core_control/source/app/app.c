/**
 * @file app.c
 * @brief application layer initialization 
 *        初始化所有 app 层任务
 * @author Davy Refss
 *
 */

#include "app.h"
#include "comm_task.h"
#include "led_task.h"
#include "motor_task.h"
#include "battery_task.h"

void app_init(void) {
    led_task_init();
    motor_task_init();
    comm_task_init();
    battery_task_init();
}
