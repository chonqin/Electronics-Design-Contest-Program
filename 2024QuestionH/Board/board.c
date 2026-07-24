/**
 * @file board.c
 * @brief 板级串口打印与延时辅助函数实现
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "bsp_uart.h"
#include "ti/driverlib/m0p/dl_core.h"

int fputc(int ch, FILE *f)
{
    // 重定向 printf 的单字符输出到底层串口。
    (void)f;
    BSP_Uart_WriteByte((uint8_t)ch);

    return ch;
}

int LOG_Debug_Out(const char *__file, const char *__func, int __line, const char *format, ...)
{
    va_list args;
    char log_buff[64] = {0};
    char buffer[512] = {0};
    char temp_buff[] = "\r\n";
    int len;

    va_start(args, format);

    // 先拼接文件名、函数名和行号，便于串口定位问题来源。
    sprintf(log_buff, "[%s Func:%s Line:%d] ", __file, __func, __line);

    // 再把前缀和格式化正文合并到统一发送缓冲区。
    strcpy(buffer, log_buff);
    len = vsnprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), format, args);

    va_end(args);

    // 统一补回车换行，保持串口日志一条一行。
    strcat(buffer, temp_buff);
    BSP_Uart_Write((uint8_t const *)buffer, (uint16_t)strlen(buffer));

    return len;
}

int lc_printf(char *format, ...)
{
    va_list args;
    char buffer[512] = {0};
    int len;

    va_start(args, format);

    // 轻量打印不加额外前缀，直接复用统一的 UART 发送链路。
    len = vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    // 格式化完成后立即透传到串口。
    BSP_Uart_Write((uint8_t const *)buffer, (uint16_t)strlen(buffer));

    return len;
}

void delay_us(int __us) { delay_cycles((CPUCLK_FREQ / 1000 / 1000) * __us); }
void delay_ms(int __ms) { delay_cycles((CPUCLK_FREQ / 1000) * __ms); }

void delay_1us(int __us) { delay_cycles((CPUCLK_FREQ / 1000 / 1000) * __us); }
void delay_1ms(int __ms) { delay_cycles((CPUCLK_FREQ / 1000) * __ms); }
