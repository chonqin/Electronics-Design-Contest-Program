/**
 * @file bsp_uart.h
 * @brief UART0 DMA 收发与 UART1 阻塞接收接口
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

/**
 * @brief UART0 DMA 接收缓冲区大小
 */
#define BSP_UART_RX_BUF_SIZE    256U

/**
 * @brief UART0 DMA 发送环形缓冲区大小
 */
#define BSP_UART_TX_BUF_SIZE    512U

/**
 * @brief 初始化 UART0 DMA 收发缓冲区和中断
 */
void BSP_Uart_Init(void);

/**
 * @brief UART0 DMA 完成中断处理入口
 */
void BSP_Uart_IRQHandler(void);

/**
 * @brief 从 UART0 DMA 接收缓冲区读取一个字节
 * @param dat 输出字节指针
 * @return 成功返回 1，无数据或参数无效返回 0
 */
int BSP_Uart_ReadByte(uint8_t *dat);

/**
 * @brief 将一段数据加入 UART0 DMA 发送队列
 * @param buf 待发送缓冲区
 * @param len 待发送字节数
 * @return 成功返回 len，队列空间不足返回 0
 */
uint16_t BSP_Uart_Write(uint8_t const *buf, uint16_t len);

/**
 * @brief 阻塞读取 UART1 的一个字节
 * @param dat 输出字节指针
 * @return 成功返回 1，参数无效返回 0
 */
int BSP_Uart1_ReadByteBlocking(uint8_t *dat);

#endif
