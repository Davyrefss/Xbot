#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATE_INIT = 0,   /* 初始化 */
    STATE_NORMAL,     /* 正常 */
    STATE_ERROR,      /* 错误 */
} running_state_t;

running_state_t system_state_get(void);
void            system_state_set(running_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_STATE_H */
