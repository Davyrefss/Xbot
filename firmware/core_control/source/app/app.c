/**
 * @file app.c
 * @brief application layer initialization 
 *        初始化所有 app 层任务
 * @author Davy Refss
 *
 */

/* include files */
#include "app.h"
#include "comm_task.h"
#include "led_task.h"
#include "motor_task.h"
#include "battery_task.h"
#include "test_task.h"
#include "console.h"

/* functions implementation */
void app_init(void) {
    led_task_init();
    motor_task_init();
    comm_task_init();
    battery_task_init();
#ifdef ORDINARY_DEBUG
    test_task_init();
#endif    
}
