/**
 * @file board.c
 * @brief Board level UART print and delay helpers.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "bsp_uart.h"
#include "ti/driverlib/m0p/dl_core.h"

int fputc(int ch, FILE *f)
{
    (void)f;
#if BOARD_TEXT_LOG_ENABLE
    uint8_t dat = (uint8_t)ch;

    (void)BSP_Uart_Write(&dat, 1U);
#else
    (void)ch;
#endif

    return ch;
}

/**
 * @brief Queue one fully formatted printf payload to UART TX buffer.
 * @param format Printf style format string.
 * @return Formatted character count, negative value on format error.
 */
int printf(const char *format, ...)
{
#if BOARD_TEXT_LOG_ENABLE
    va_list args;
    char buffer[512] = {0};
    int len;
    uint16_t send_len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len <= 0) {
        return len;
    }

    send_len = (uint16_t)len;
    if (send_len >= sizeof(buffer)) {
        send_len = (uint16_t)(sizeof(buffer) - 1U);
    }

    /* Queue the full payload once, then let UART FIFO requests drive DMA. */
    (void)BSP_Uart_Write((uint8_t const *)buffer, send_len);

    return len;
#else
    (void)format;
    return 0;
#endif
}

int LOG_Debug_Out(const char *__file, const char *__func, int __line, const char *format, ...)
{
#if BOARD_TEXT_LOG_ENABLE
    va_list args;
    char log_buff[64] = {0};
    char buffer[512] = {0};
    char temp_buff[] = "\r\n";
    int len;

    va_start(args, format);

    sprintf(log_buff, "[%s Func:%s Line:%d] ", __file, __func, __line);
    strcpy(buffer, log_buff);
    len = vsnprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), format, args);

    va_end(args);

    strcat(buffer, temp_buff);
    BSP_Uart_Write((uint8_t const *)buffer, (uint16_t)strlen(buffer));

    return len;
#else
    (void)__file;
    (void)__func;
    (void)__line;
    (void)format;
    return 0;
#endif
}

void delay_us(int __us) { delay_cycles((CPUCLK_FREQ / 1000 / 1000) * __us); }
void delay_ms(int __ms) { delay_cycles((CPUCLK_FREQ / 1000) * __ms); }

void delay_1us(int __us) { delay_cycles((CPUCLK_FREQ / 1000 / 1000) * __us); }
void delay_1ms(int __ms) { delay_cycles((CPUCLK_FREQ / 1000) * __ms); }
