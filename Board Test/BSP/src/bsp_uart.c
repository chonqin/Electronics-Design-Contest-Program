/**
 * @file bsp_uart.c
 * @brief UART0 环形缓冲接收与发送实现
 */

#include <string.h>

#include "bsp_uart.h"

/* 采用 256 字节缓冲，利用 8 位下标自然回卷 */
static volatile uint8_t rx_buf[BSP_UART_RX_BUF_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;

/**
 * @brief 向环形缓冲写入一个字节
 * @param dat 待写入字节
 */
static void uart_rx_push(uint8_t dat)
{
    uint8_t next = (uint8_t)(rx_head + 1U);

    if (next == rx_tail) {
        return;
    }

    rx_buf[rx_head] = dat;
    rx_head = next;
}

/**
 * @brief 从 UART 硬件 FIFO 中搬运数据到环形缓冲
 */
static void uart_rx_drain_fifo(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
        uart_rx_push((uint8_t)DL_UART_Main_receiveData(UART_0_INST));
    }
}

void BSP_Uart_Init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
}

void BSP_Uart_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            uart_rx_drain_fifo();
            break;
        default:
            break;
    }
}

uint16_t BSP_Uart_Available(void)
{
    return (uint16_t)((uint8_t)(rx_head - rx_tail));
}

int BSP_Uart_ReadByte(uint8_t *dat)
{
    if ((dat == NULL) || (rx_head == rx_tail)) {
        return 0;
    }

    *dat = rx_buf[rx_tail];
    rx_tail = (uint8_t)(rx_tail + 1U);

    return 1;
}

uint16_t BSP_Uart_Read(uint8_t *buf, uint16_t len)
{
    uint16_t cnt = 0U;

    if ((buf == NULL) || (len == 0U)) {
        return 0U;
    }

    while ((cnt < len) && (rx_head != rx_tail)) {
        buf[cnt++] = rx_buf[rx_tail];
        rx_tail = (uint8_t)(rx_tail + 1U);
    }

    return cnt;
}

void BSP_Uart_FlushRx(void)
{
    rx_head = 0U;
    rx_tail = 0U;
}

int BSP_Uart_WriteByte(uint8_t dat)
{
    while (DL_UART_Main_fillTXFIFO(UART_0_INST, &dat, 1U) == 0U) {
        ;
    }

    return 1;
}

uint16_t BSP_Uart_Write(uint8_t const *buf, uint16_t len)
{
    uint16_t cnt = 0U;
    uint32_t wr;

    if ((buf == NULL) || (len == 0U)) {
        return 0U;
    }

    while (cnt < len) {
        wr = DL_UART_Main_fillTXFIFO(UART_0_INST, (uint8_t *)&buf[cnt], (uint32_t)(len - cnt));
        cnt = (uint16_t)(cnt + (uint16_t)wr);

        if (wr == 0U) {
            while (DL_UART_Main_fillTXFIFO(UART_0_INST, (uint8_t *)&buf[cnt], 1U) == 0U) {
                ;
            }
            cnt++;
        }
    }

    return cnt;
}

uint16_t BSP_Uart_WriteString(char const *str)
{
    if (str == NULL) {
        return 0U;
    }

    return BSP_Uart_Write((uint8_t const *)str, (uint16_t)strlen(str));
}
