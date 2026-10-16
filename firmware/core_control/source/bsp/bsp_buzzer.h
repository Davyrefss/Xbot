/**
 * @file bsp_buzzer.h
 * @author Davyrefss
 * @brief 
 *
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

/* include files */
#include "main.h"

/* defines */
#define BUZZER_GPIO_PORT        BUZZER_GPIO_Port
#define BUZZER_PIN              BUZZER_Pin
#define BUZZER_ACTIVE_LEVEL     GPIO_PIN_SET
#define BUZZER_INACTIVE_LEVEL   GPIO_PIN_RESET 


/* typedefs */
typedef struct {
    GPIO_TypeDef* buzzer_gpio_port;
    uint16_t buzzer_gpio_pin;
} buzzer_gpio_config_t;

/* functions */
void buzzer_on(buzzer_gpio_config_t* buzzer_config);
void buzzer_off(buzzer_gpio_config_t* buzzer_config);



#endif /* BSP_BUZZER_H */
