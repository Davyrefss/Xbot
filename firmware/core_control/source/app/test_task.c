/**
 * @file test_task.h
 * @author Davyrefss
 * @brief test task
 *
 */

/* include files */
#include "test_task.h"
#include "test_service.h"
#include "bsp_usart.h"
#include "bsp_motor.h"
#include "osal.h"
#include <stdio.h>
#include <stdint.h>

/* defines */
#define TEST_TASK_PERIOD_MS     20u                 /* 任务周期 */
#define TEST_TASK_STACK_WORDS   128u                /* 栈大小128*4字节 */
#define TEST_TASK_PRIORITY      OSAL_PRIO_NORMAL    /* 任务优先级 */

/* static variables */
static osal_task_handle_t test_task_handle = NULL;

/* functions implementation */
/**
 * @brief Test task function
 */
static void test_task(void *arg) {
    (void)arg;
    uint8_t M1 = 1;
    uint16_t count;
    osal_wake_time_t wake;
    osal_wake_time_init(&wake);

    for (;;) {
        count = bsp_motor_get_encoder((motor_id_t)M1);
        printf("%d\r\n", count);

        osal_delay_until(&wake, TEST_TASK_PERIOD_MS);
    }
}

/**
 * @brief test task initialization
 */
void test_task_init() {
    if (!osal_task_create("test_task", TEST_TASK_STACK_WORDS, TEST_TASK_PRIORITY, 
                        test_task, NULL, &test_task_handle)) {
        printf("Failed to create test task\n");
    }
}

