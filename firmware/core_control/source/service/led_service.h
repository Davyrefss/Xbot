#ifndef LED_SERVICE_H
#define LED_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* typedefs */
/* LED 行为模式：Service 只负责“怎么亮”，不关心“为什么这么亮” */
typedef enum {
    LED_MODE_OFF = 0,       /* 常灭 */
    LED_MODE_ON,            /* 常亮 */
    LED_MODE_BLINK_FAST,    /* 快闪(200ms) */
    LED_MODE_BLINK_SLOW,    /* 慢闪(500ms) */
} led_mode_t;

/* functions declaration */
void led_service_init(void);
void led_service_set_mode(led_mode_t mode);
void led_service_poll(void);   /* 由 App 周期调用，内部按当前模式驱动 GPIO */

#ifdef __cplusplus
}
#endif

#endif /* LED_SERVICE_H */
