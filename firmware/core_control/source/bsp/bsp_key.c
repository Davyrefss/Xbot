/**
 * @file bsp_key.h
 * @author Davyrefss
 * @brief key bsp
 *
 */

/* include files */
#include "bsp_key.h"

/* functions implementation */
key_state_t key_state_get(key_gpio_config_t* key_config) {
    return (key_state_t)HAL_GPIO_ReadPin(key_config->key_gpio_port, 
        key_config->key_gpio_pin
    );
}

