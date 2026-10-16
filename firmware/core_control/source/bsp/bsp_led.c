/**
 * @file bsp_led.c
 * @author Davyrefss
 * @brief 
 *
 */

/* include files */ 
#include "bsp_led.h"

/**
 * @brief 打开LED
 */
void led_on(led_gpio_config_t* led_config){
    HAL_GPIO_WritePin(led_config->led_gpio_port, 
        led_config->led_gpio_pin, 
        LED_ACTIVE_LEVEL
    );
}

/**
 * @brief 关闭LED
 */
void led_off(led_gpio_config_t* led_config){
    HAL_GPIO_WritePin(led_config->led_gpio_port, 
        led_config->led_gpio_pin, 
        LED_INACTIVE_LEVEL
    );
}

/**
 * @brief 反转LED状态
 */
void led_toggle(led_gpio_config_t* led_config){
    HAL_GPIO_TogglePin(led_config->led_gpio_port, 
        led_config->led_gpio_pin
    );
}