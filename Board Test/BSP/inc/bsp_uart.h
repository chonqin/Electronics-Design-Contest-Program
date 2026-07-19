/**
 * @file bsp_uart.h
 * @brief UART0 环形缓冲接收接口
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include "ti_msp_dl_config.h"

/**
 * @brief UART 接收缓冲区大小
 */
#define BSP_UART_RX_BUF_SIZE    256U

/**
 * @brief 初始化 UART0 接收环形缓冲
 */
void BSP_Uart_Init(void);

/**
 * @brief UART0 中断处理函数
 */
void BSP_Uart_IRQHandler(void);

/**
 * @brief 获取当前缓冲区内可读字节数
 * @return 可读字节数
 */
uint16_t BSP_Uart_Available(void);

/**
 * @brief 读取单个接收字节
 * @param dat 输出字节地址
 * @return 读取成功返回 1，缓冲为空返回 0
 */
int BSP_Uart_ReadByte(uint8_t *dat);

/**
 * @brief 批量读取接收数据
 * @param buf 目标缓冲区
 * @param len 期望读取长度
 * @return 实际读取长度
 */
uint16_t BSP_Uart_Read(uint8_t *buf, uint16_t len);

/**
 * @brief 清空接收缓冲区
 */
void BSP_Uart_FlushRx(void);

/**
 * @brief 发送单个字节
 * @param dat 待发送字节
 * @return 发送成功返回 1
 */
int BSP_Uart_WriteByte(uint8_t dat);

/**
 * @brief 批量发送数据
 * @param buf 待发送数据
 * @param len 待发送长度
 * @return 实际发送长度
 */
uint16_t BSP_Uart_Write(uint8_t const *buf, uint16_t len);

/**
 * @brief 发送字符串
 * @param str 待发送字符串
 * @return 实际发送字符数
 */
uint16_t BSP_Uart_WriteString(char const *str);

#endif
