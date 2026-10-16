/**
 * @file console.c
 * @author Davy Refss
 * @brief printf 重定向到 USART1（调试/日志设施，不属于 BSP 驱动本身）
 */

/* include files */
#include "bsp_usart.h"

/* functions implementation */
/* ================= printf / scanf 重定向 ================= */
/**
 * @brief printf 底层单字节输出, 由 syscalls.c 的 _write 调用
 * @note  调用方需自行使用 "\r\n" 换行(项目内 printf 均已带 \r\n)
 */
int __io_putchar(int ch){
    bsp_usart1_send_byte((uint8_t)ch);
    return ch;
}
