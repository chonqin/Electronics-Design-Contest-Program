/**
 * @file board.c
 * @brief 板级串口打印与延时辅助函数实现
 */

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "board.h"
#include "bsp_uart.h"
#include "ti/driverlib/m0p/dl_core.h"

int fputc(int ch, FILE *f)
{
    // 重定向 printf 的单字符输出
    (void)f;
    BSP_Uart_WriteByte((uint8_t)ch);

    return ch;
}

int LOG_Debug_Out(const char *__file, const char *__func, int __line, const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // 拼接日志前缀信息
    char log_buff[64] = {0};
    sprintf(log_buff, "[%s Func:%s Line:%d] ", __file, __func, __line);

    // 创建足够大的缓冲区存储格式化后的字符串
    char buffer[512] = {0};
    strcpy(buffer, log_buff);
    int len = vsnprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), format, args);

    va_end(args);

    // 发送格式化后的字符串
    char temp_buff[] = "\r\n";
    strcat(buffer, temp_buff);
    BSP_Uart_Write((uint8_t const *)buffer, (uint16_t)strlen(buffer));

    return len;
}

int lc_printf(char *format, ...)
{
    va_list args;
    va_start(args, format);

    // 创建足够大的缓冲区存储格式化后的字符串
    char buffer[512] = {0};
    int len = vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    // 发送格式化后的字符串
    BSP_Uart_Write((uint8_t const *)buffer, (uint16_t)strlen(buffer));

    return len;
}

void delay_us(int __us) { delay_cycles((CPUCLK_FREQ / 1000 / 1000) * __us); }
void delay_ms(int __ms) { delay_cycles((CPUCLK_FREQ / 1000) * __ms); }

void delay_1us(int __us) { delay_cycles((CPUCLK_FREQ / 1000 / 1000) * __us); }
void delay_1ms(int __ms) { delay_cycles((CPUCLK_FREQ / 1000) * __ms); }
