/**
 * @brief OSAL (Operating System Abstraction Layer) implementation for FreeRTOS
 * @author Davy Refss
 *
 */

/* include files */ 
#include <stdint.h>
#include <stdbool.h>
#include "osal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"

/* functions implementation */
/**
 * @brief create a new TaskCreate
 * @return bool 
 */
bool osal_task_create(const char* name, uint16_t stack_words, uint8_t priority,
                      osal_task_func_t func, void* arg, osal_task_handle_t* out_handle) {
    BaseType_t ret = xTaskCreate((TaskFunction_t)func,
                                 name,
                                 (configSTACK_DEPTH_TYPE)stack_words,
                                 arg,
                                 (UBaseType_t)priority,
                                 (TaskHandle_t *)out_handle);
    return (ret == pdPASS);
}

/**
 * @brief Get current system time in milliseconds
 * @return uint32_t Current time in milliseconds
 */
uint32_t osal_time_get_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * (TickType_t)portTICK_PERIOD_MS);
}

/**
 * @brief Delay execution for a specified number of milliseconds
 * @param ms Number of milliseconds to delay
 */
void osal_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Initialize wake time reference
 */
void osal_wake_time_init(osal_wake_time_t *w) {
    *w = (osal_wake_time_t)xTaskGetTickCount();
}

/**
 * @brief Delay until a specified time
 * @param w Pointer to wake time reference
 * @param period_ms Period in milliseconds
 */
void osal_delay_until(osal_wake_time_t *w, uint32_t period_ms)
{
    vTaskDelayUntil((TickType_t *)w, pdMS_TO_TICKS(period_ms));
}
