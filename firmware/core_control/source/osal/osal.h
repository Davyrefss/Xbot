#ifndef OSAL_H
#define OSAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* typedefs */
/* ---------- 任务 ---------- */
typedef void (*osal_task_func_t)(void* arg);
typedef void *osal_task_handle_t;
/* ---------- 时间 / 延时 ---------- */
/* 周期调度用的唤醒时间句柄，对上层不透明（内部是 FreeRTOS tick） */
typedef uint32_t osal_wake_time_t;

/* defines */
/* 优先级抽象：上层只认这些符号，不认 FreeRTOS 的 tskIDLE_PRIORITY。
 * 取值需 < configMAX_PRIORITIES(当前=7) */
#define OSAL_PRIO_IDLE     0u
#define OSAL_PRIO_LOW      1u
#define OSAL_PRIO_NORMAL   2u
#define OSAL_PRIO_HIGH     3u
#define OSAL_PRIO_HIGHEST  4u

/* functions declaration */
bool osal_task_create(const char* name,
                      uint16_t stack_words,
                      uint8_t  priority,
                      osal_task_func_t func,
                      void* arg,
                      osal_task_handle_t* out_handle);

uint32_t osal_time_get_ms(void);                        /* 上电后单调递增毫秒计数 */
void     osal_delay_ms(uint32_t ms);                    /* 相对延时 */
void     osal_wake_time_init(osal_wake_time_t *w);      /* 初始化唤醒时间 */
void     osal_delay_until(osal_wake_time_t *w, uint32_t period_ms); /* 固定周期延时 */

#ifdef __cplusplus
}
#endif

#endif /* OSAL_H */