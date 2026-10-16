/**
 * @file bsp_buzzer.h
 * @author Davyrefss
 * @brief 
 *
 */

/* include files */ 
#include "bsp_buzzer.h" 

/* static variables */


/* functions implementation */
/**
 * @brief buzzer on
 * 
 * @param buzzer_config 
 */
void buzzer_on(buzzer_gpio_config_t* buzzer_config) {
    HAL_GPIO_WritePin(buzzer_config->buzzer_gpio_port, 
        buzzer_config->buzzer_gpio_pin, 
        BUZZER_ACTIVE_LEVEL
    );
}

/**
 * @brief buzzer off
 * 
 * @param buzzer_config 
 */
void buzzer_off(buzzer_gpio_config_t* buzzer_config) {
    HAL_GPIO_WritePin(buzzer_config->buzzer_gpio_port, 
        buzzer_config->buzzer_gpio_pin, 
        BUZZER_INACTIVE_LEVEL
    );
}

