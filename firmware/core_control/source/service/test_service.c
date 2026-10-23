/**
 * @file test_service.c
 * @author Davyrefss
 * @brief test service
 *
 */

/* include files */
#include "test_service.h" 
#include "bsp_motor.h"
#include <stdint.h>

/* defines */
#define MOTOR_DUTY_MAX_TEST     1000

/* functions implementation */
/**
 * @brief Set motor duty cycle for testing
 * 
 * @param id Motor ID
 * @param duty Duty cycle value
 */
void set_duty_test(motor_id_t id, int16_t duty) {
    uint16_t d, in1, in2;

    if (duty > MOTOR_DUTY_MAX_TEST) {
        duty = MOTOR_DUTY_MAX_TEST;
    }
    if (duty < -MOTOR_DUTY_MAX_TEST) {
        duty = -MOTOR_DUTY_MAX_TEST;
    }
    if (duty > 0) {
        d = (uint16_t)((uint32_t)duty * BSP_MOTOR_PWM_PERIOD / MOTOR_DUTY_MAX_TEST);
        in1 = d; in2 = 0;
    } else if (duty < 0) {
        d = (uint16_t)((uint32_t)duty * BSP_MOTOR_PWM_PERIOD / MOTOR_DUTY_MAX_TEST);
        in1 = 0; in2 = d;
    } else {
        in1 = 0; in2 = 0;
    }
    bsp_motor_set_duty(id, in1, in2 );
}



