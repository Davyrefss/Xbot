/**
 * @file bsp_key.h
 * @author Davyrefss
 * @brief key bsp
 *
 */

#ifndef BSP_KEY_H
#define BSP_KEY_H

/* include files */
#include "main.h"

/* defines */
#define KEY1_GPIO_PORT  KEY1_GPIO_Port
#define KEY1_GPIO_PIN   KEY1_Pin

/* typedefs */
typedef struct {
    GPIO_TypeDef *key_gpio_port;
    uint16_t key_gpio_pin;
} key_gpio_config_t;

typedef enum {
    KEY_PRESS = 0,
    KEY_RELEASE = 1,
} key_state_t;

/* functions declaration */
key_state_t key_state_get(key_gpio_config_t* key_config);

#endif /* BSP_KEY_H */

