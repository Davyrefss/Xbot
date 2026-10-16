#ifndef BSP_USART_H
#define BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* include files */
#include <stdint.h>
#include "usart.h"

/* functions declaration*/
/* USART1 驱动接口 */
void bsp_usart1_init(void);                                 /* 启动中断接收(在 MX_USART1_UART_Init 后调用) */
uint16_t bsp_usart1_available(void);                        /* 接收缓冲中可读字节数 */
int bsp_usart1_read(uint8_t *byte);                         /* 非阻塞读 1 字节, 读到返回 1, 否则 0 */
void bsp_usart1_send_byte(uint8_t byte);                    /* 阻塞发送 1 字节 */
void bsp_usart1_send(const uint8_t *data, uint16_t len);    /* 阻塞发送 len 字节 */

#ifdef __cplusplus
}
#endif

#endif  /* BSP_USART_H */