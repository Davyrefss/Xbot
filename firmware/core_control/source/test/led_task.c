#include "led_task.h"
#include "led.h"
#include "FreeRTOS.h"
#include "main.h"
#include "portmacro.h"
#include "task.h"
#include "stdio.h"
#include "config.h"
#include <stdint.h>

/* const values*/
static const uint16_t led_task_period_ms = 20;
/**
 * led_blink_period_st_init_ms -> 初始化时，100ms闪烁
 * led_blink_period_st_nor_ms -> normal正常工作时，500ms闪烁
 */
static const uint16_t led_blink_period_st_init_ms = 200;
static const uint16_t led_blink_period_st_nor_ms = 500;

/* static variables */
static TaskHandle_t led_task_handle = NULL;     /* 任务句柄 */
static led_gpio_config_t led_gpio_config = {
    .led_gpio_port = LED_GPIO_Port,
    .led_gpio_pin = LED_Pin,
};

/* static functions */
/**
 *@brief LED闪烁
 *@param period_ms 闪烁的周期ms
 */
static void led_blink(uint16_t period_ms){
    static TickType_t last_time = 0;
    TickType_t cur_time = xTaskGetTickCount();
    if ((cur_time - last_time) >= pdMS_TO_TICKS(period_ms)) {
        led_toggle(&led_gpio_config);
        last_time = cur_time;
    }
}

/**
 * @brief LED任务
 * @param argument 任务参数
 */
static void led_task(){
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        switch (running_state) {
            case STATE_INIT:
                led_blink(led_blink_period_st_init_ms);
                break;
            case STATE_NORMAL:
                led_blink(led_blink_period_st_nor_ms);
                break;
            case STATE_ERROR:
                led_on(&led_gpio_config);
                break;
            case STATE_IDLE:
                led_off(&led_gpio_config);
                break;
            default:
                break;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(led_task_period_ms));     /* 周期时间:20ms */
    }
}

/**
 * @brief LED任务init
 */
void led_task_init(void){
    BaseType_t ret = xTaskCreate(led_task,      /* 任务函数 */
                                "led_task",         /* 任务名 */
                                128,            /* 栈深度，单位是字(4个字节(32位平台)) */       
                                NULL,           /* 任务参数 */
                                tskIDLE_PRIORITY + 1,   /* 任务优先级(0+1),范围0~55，越大越优先 */
                                &led_task_handle);      /* 任务句柄 */                 
    if (ret != pdPASS) {
        printf("create led_task failed\r\n");
    }
}
