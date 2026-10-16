/**
 * @file led_task.c
 * @brief LED 任务：App 层，负责编排 system_state + led_service
 */

/* include files */ 
#include "led_task.h"
#include "led_service.h"
#include "battery_service.h"
#include "system_state.h"
#include "osal.h"
#include <stddef.h>
#include <stdio.h>

/* defines */
#define LED_TASK_PERIOD_MS    20u               /* 20ms period */   
#define LED_TASK_STACK_WORDS  128u              /* stack size: 128 words */       
#define LED_TASK_PRIORITY     OSAL_PRIO_LOW     /* priority: low */

/* static variables */
static osal_task_handle_t led_task_handle = NULL;

/* functions implementation */
/**
 * 将运行状态转换为对应的LED模式
 * LED 语义约定:
 *   常亮        -> 上电初始化 / 未连接上位机 (STATE_INIT)
 *   500ms 慢闪  -> 正常运行心跳 (STATE_NORMAL, 收到任意有效命令后进入)
 *   200ms 快闪  -> 低电量告警 (任务循环里由 battery_service 覆盖, 优先级最高)
 *   常灭        -> 错误 (STATE_ERROR)
 * @param state 系统运行状态
 * @return 对应的LED模式
 */
static led_mode_t state_to_led_mode(running_state_t state) {
    switch (state) {
    case STATE_INIT:   return LED_MODE_ON;          /* 初始化/未连上位机：常亮 */
    case STATE_NORMAL: return LED_MODE_BLINK_SLOW;  /* 正常：500ms 慢闪(心跳) */
    case STATE_ERROR:  return LED_MODE_OFF;         /* 错误：常灭 */
    default:           return LED_MODE_OFF;
    }
}

/**
 * LED任务主函数
 * @param arg 任务参数
 */
static void led_task(void *arg) {
    (void)arg;
    osal_wake_time_t wake;

    osal_wake_time_init(&wake);
    led_service_init();

    for (;;) {
        led_mode_t mode = state_to_led_mode(system_state_get());
        if (battery_service_is_low_battery()) {
            mode = LED_MODE_BLINK_FAST;   /* 低电量告警: 快闪 */
        }
        led_service_set_mode(mode);
        led_service_poll();
        osal_delay_until(&wake, LED_TASK_PERIOD_MS);   /* 固定 20ms 周期 */
    }
}


void led_task_init(void)
{
    if (!osal_task_create("led_task", LED_TASK_STACK_WORDS, LED_TASK_PRIORITY,
                          led_task, NULL, &led_task_handle)) {
        printf("create led_task failed\r\n");
    }
}
