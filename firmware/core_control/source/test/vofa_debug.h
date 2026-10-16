#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 发送一帧 VOFA+ JustFloat 波形数据(4x 电机转速 RPM) */
void vofa_debug_send(void);

#ifdef __cplusplus
}
#endif

#endif  /* VOFA_DEBUG_H */
