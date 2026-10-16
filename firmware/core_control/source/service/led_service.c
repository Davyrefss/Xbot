/**
 * @file led_service.c
 * @author Davy Refss
 *
 */

/* include files */
#include "led_service.h"
#include "bsp_led.h"
#include "osal.h"

#define LED_BLINK_FAST_MS  200u
#define LED_BLINK_SLOW_MS  500u

/* static variables */
static led_gpio_config_t led_gpio = {
    .led_gpio_port = LED_GPIO_PORT,
    .led_gpio_pin  = LED_GPIO_PIN,
};
static led_mode_t cur_mode = LED_MODE_OFF;
static uint32_t   last_ms  = 0u;

/* functions implementation */
/**
 * @brief LED服务初始化函数
 *        初始化LED服务并设置初始状态
 */
void led_service_init(void) {
    cur_mode = LED_MODE_OFF;
    last_ms  = osal_time_get_ms();
    led_off(&led_gpio);
}

/**
 * @brief LED服务设置模式函数
 *        设置LED的工作模式
 */
void led_service_set_mode(led_mode_t mode) {
    if (mode != cur_mode) {
        cur_mode = mode;
        last_ms = osal_time_get_ms();   
    }
}

/**
 * @brief LED服务轮询函数
 *        定期检查LED状态并更新LED灯的亮灭状态
 */
void led_service_poll(void) {
    uint32_t now = osal_time_get_ms();

    switch (cur_mode) {
    case LED_MODE_ON:
        led_on(&led_gpio);
        break;
    case LED_MODE_OFF:
        led_off(&led_gpio);
        break;
    case LED_MODE_BLINK_FAST:
        if ((now - last_ms) >= LED_BLINK_FAST_MS) {
            led_toggle(&led_gpio);
            last_ms = now;
        }
        break;
    case LED_MODE_BLINK_SLOW:
        if ((now - last_ms) >= LED_BLINK_SLOW_MS) {
            led_toggle(&led_gpio);
            last_ms = now;
        }
        break;
    default:
        break;
    }
}
