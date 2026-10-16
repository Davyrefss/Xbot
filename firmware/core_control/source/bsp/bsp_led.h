/**
 * @file bsp_led.h
 * @author Davyrefss
 * @brief led bsp
 *
 */

#ifndef BSP_LED_H
#define BSP_LED_H

/* include files */
#include "main.h"

/* defines */
#define LED_GPIO_PORT       LED_GPIO_Port
#define LED_GPIO_PIN        LED_Pin
#define LED_SW_GPIO_PORT    LED_SW_GPIO_Port
#define LED_SW_GPIO_PIN     LED_SW_Pin
#define LED_ACTIVE_LEVEL    GPIO_PIN_SET
#define LED_INACTIVE_LEVEL  GPIO_PIN_RESET

/* typedef defines */
typedef struct {
    GPIO_TypeDef* led_gpio_port;
    uint16_t led_gpio_pin;
} led_gpio_config_t;

/* exported functions */
void led_on(led_gpio_config_t* led_config);
void led_off(led_gpio_config_t* led_config);
void led_toggle(led_gpio_config_t* led_config);




#endif /* BSP_LED_H */
