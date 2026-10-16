#include "motor_task.h"
#include "motor_service.h"
#include "osal.h"
#include <stdio.h>

#define MOTOR_TASK_PERIOD_MS    10u
#define MOTOR_TASK_STACK_WORDS  256u
#define MOTOR_TASK_PRIORITY     OSAL_PRIO_HIGH

static osal_task_handle_t motor_task_handle = NULL;

static void motor_task(void *arg)
{
    (void)arg;
    osal_wake_time_t wake;
    osal_wake_time_init(&wake);

    motor_service_init();

    for (;;) {
        motor_service_poll();                                           /* M/T测速 + 速度环PID */
        osal_delay_until(&wake, MOTOR_TASK_PERIOD_MS);     /* 固定 10ms */

    }
}

void motor_task_init(void)
{
    if (!osal_task_create("motor_task", MOTOR_TASK_STACK_WORDS, MOTOR_TASK_PRIORITY,
                          motor_task, NULL, &motor_task_handle)) {
        printf("create motor_task failed\r\n");
    }
}
