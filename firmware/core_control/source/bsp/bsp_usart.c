/**
 * @file bsp_usart.c
 * @author Davyrefss
 * @brief USART1 串口驱动
 *
 * USART1: 115200-8-N-1, TX=PA9, RX=PA10 (由 CubeMX 在 Core/Src/usart.c 中初始化)
 * 接收: HAL_UART_Receive_IT 单字节中断接收 -> 回调中存入环形缓冲
 * 发送: HAL_UART_Transmit 阻塞发送
 * 重定向: 定义 __io_putchar / __io_getchar, 使 printf / scanf 走 USART1
 *         (Core/Src/syscalls.c 的 _write/_read 会调用这两个弱符号)
 *
 * 注意: __io_getchar 与通信协议共享接收缓冲, 勿同时使用 scanf 与协议帧
 */

/* include files */
#include "bsp_usart.h"

/* 接收环形缓冲(2 的幂, 用掩码取模) */
#define BSP_USART1_RX_BUF_SIZE  256u
#define BSP_USART1_RX_BUF_MASK  (BSP_USART1_RX_BUF_SIZE - 1u)

/* static variables */
static uint8_t  rx_ring[BSP_USART1_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;   /* 单调递增写计数, 中断中修改 */
static volatile uint16_t rx_tail = 0;   /* 单调递增读计数, 任务中修改 */
static uint8_t  rx_byte = 0;            /* 中断接收的单字节缓冲 */

/* functions implementation */
/**
 * @brief USART1 接收完成回调(中断上下文)
 * @note  每收到 1 字节触发一次: 存入环形缓冲并重新接收
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART1) {
        uint16_t next = (uint16_t)(rx_head + 1u);
        /* 有空间才写入, 满则丢弃该字节 */
        if ((uint16_t)(next - rx_tail) < BSP_USART1_RX_BUF_SIZE) {
            /* 数组索引始终在0~255 */
            rx_ring[rx_head & BSP_USART1_RX_BUF_MASK] = rx_byte;
            rx_head = next;
        }
        /* 继续接收下一字节 */
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/**
 * @brief 接收错误回调(中断上下文)
 * @note  过载(ORE)等错误会导致 HAL 停止 RX 中断, 这里清除错误标志并重新武装,
 *        防止接收中断卡死
 *  ORE --Overrun Error，过载错误。接收数据寄存器还没被读走，新数据又来了，导致数据丢失。
 *  NE --Noise Error，噪声错误。在接收时检测到噪声。
 *  FE --Framing Error，帧错误。停止位不对等。
 *  PE --Parity Error，奇偶校验错误。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART1) {
        /* 读 SR + DR 以清除 ORE/NE/FE/PE 标志 */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        /* 重新武装接收 */
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/**
 * @brief 启动 USART1 中断接收
 * @note  需在 MX_USART1_UART_Init() 之后调用
 */
void bsp_usart1_init(void){
    rx_head = 0;
    rx_tail = 0;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/**
 * @brief 接收缓冲中可读字节数
 */
uint16_t bsp_usart1_available(void){
    return (uint16_t)(rx_head - rx_tail);
}

/**
 * @brief 非阻塞读 1 字节
 * @param byte 输出参数
 * @return 读到返回 1, 无数据返回 0
 */
int bsp_usart1_read(uint8_t *byte){
    if (rx_head != rx_tail) {
        *byte = rx_ring[rx_tail & BSP_USART1_RX_BUF_MASK];
        rx_tail++;
        return 1;
    }
    return 0;
}

/**
 * @brief 阻塞发送 1 字节
 */
void bsp_usart1_send_byte(uint8_t byte){
    HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
}

/**
 * @brief 阻塞发送 len 字节
 */
void bsp_usart1_send(const uint8_t *data, uint16_t len){
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, HAL_MAX_DELAY);
}
