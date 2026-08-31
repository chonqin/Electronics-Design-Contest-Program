/**
 * @file bsp_motor.c
 * @brief QD4310 绝对角度 DMA 事务及反馈解析实现。
 */

#include "bsp_motor.h"

#include <stddef.h>
#include <stdint.h>

#define BSP_MOTOR_TX_LEN          5U
#define BSP_MOTOR_RX_LEN          10U
#define BSP_MOTOR_ID_MAX          0x0FU
#define BSP_MOTOR_CRC_POLY        0x07U
#define BSP_MOTOR_S16_SCALE       32767.0f
#define BSP_MOTOR_U16_SCALE       65535.0f
#define BSP_MOTOR_CURRENT_MAX     10.0f
#define BSP_MOTOR_SPEED_MAX       1000.0f
#define BSP_MOTOR_TWO_PI          6.28318530717958647692f
#define BSP_MOTOR_TEST_ANGLE      0.4f
#define BSP_MOTOR_TEST_DEBOUNCE_MS 20U

typedef enum
{
    BSP_MOTOR_CMD_ENABLE = 0x01,
    BSP_MOTOR_CMD_DISABLE = 0x02,
    BSP_MOTOR_CMD_ANGLE = 0x05
} bsp_motor_cmd_t;

typedef enum
{
    BSP_MOTOR_TEST_ENABLE = 0U,
    BSP_MOTOR_TEST_WAIT_ENABLE,
    BSP_MOTOR_TEST_WAIT_ZERO,
    BSP_MOTOR_TEST_READY,
    BSP_MOTOR_TEST_WAIT_ANGLE,
    BSP_MOTOR_TEST_ERROR
} bsp_motor_test_state_t;

static uint8_t BSP_MOTOR_Crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0U;
    uint32_t i;
    uint8_t bit;

    /* 按手册规定的多项式 0x07 和初值 0x00 计算 CRC8。 */
    for (i = 0U; i < len; ++i)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ BSP_MOTOR_CRC_POLY);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}

static uint8_t BSP_MOTOR_IsValid(const bsp_motor_t *dev)
{
    if ((dev == NULL) || (dev->uart == NULL) ||
        (dev->uart->Instance == NULL) || (dev->uart->hdmarx == NULL) ||
        (dev->uart->hdmatx == NULL))
    {
        return 0U;
    }
    if ((dev->id > BSP_MOTOR_ID_MAX) || (dev->timeout == 0U))
    {
        return 0U;
    }

    return 1U;
}

static float BSP_MOTOR_DecodeS16(uint8_t low, uint8_t high, float full)
{
    uint16_t word = (uint16_t)low | ((uint16_t)high << 8U);
    int16_t raw;

    if (word == 0x8000U)
    {
        return -full;
    }

    raw = (int16_t)word;
    return (float)raw * full / BSP_MOTOR_S16_SCALE;
}

static float BSP_MOTOR_DecodeU16(uint8_t low, uint8_t high, float full)
{
    uint16_t raw = (uint16_t)low | ((uint16_t)high << 8U);

    return (float)raw * full / BSP_MOTOR_U16_SCALE;
}

static void BSP_MOTOR_ResetTransfer(bsp_motor_t *dev)
{
    dev->active = 0U;
    dev->tx_done = 0U;
    dev->rx_done = 0U;
    dev->dma_error = 0U;
}

static bsp_motor_err_t BSP_MOTOR_Start(bsp_motor_t *dev,
                                       bsp_motor_cmd_t cmd, uint16_t value)
{
    HAL_StatusTypeDef hal;

    if (BSP_MOTOR_IsValid(dev) == 0U)
    {
        return BSP_MOTOR_ERR_ARG;
    }
    if (dev->active != 0U)
    {
        return BSP_MOTOR_BUSY;
    }

    dev->tx[0] = dev->id;
    dev->tx[1] = (uint8_t)cmd;
    dev->tx[2] = (uint8_t)(value & 0xFFU);
    dev->tx[3] = (uint8_t)(value >> 8U);
    dev->tx[4] = BSP_MOTOR_Crc8(dev->tx, BSP_MOTOR_TX_LEN - 1U);
    dev->start_ms = HAL_GetTick();
    dev->tx_done = 0U;
    dev->rx_done = 0U;
    dev->dma_error = 0U;
    dev->active = 1U;

    /* 先挂起接收，避免电机在发送结束后立即回复而丢失首字节。 */
    __HAL_UART_CLEAR_OREFLAG(dev->uart);
    hal = HAL_UART_Receive_DMA(dev->uart, dev->rx, BSP_MOTOR_RX_LEN);
    if (hal != HAL_OK)
    {
        BSP_MOTOR_ResetTransfer(dev);
        if (hal == HAL_BUSY)
        {
            return BSP_MOTOR_BUSY;
        }
        return BSP_MOTOR_ERR_RX;
    }

    hal = HAL_UART_Transmit_DMA(dev->uart, dev->tx, BSP_MOTOR_TX_LEN);
    if (hal != HAL_OK)
    {
        (void)HAL_UART_DMAStop(dev->uart);
        BSP_MOTOR_ResetTransfer(dev);
        if (hal == HAL_BUSY)
        {
            return BSP_MOTOR_BUSY;
        }
        return BSP_MOTOR_ERR_TX;
    }

    return BSP_MOTOR_OK;
}

bsp_motor_err_t BSP_MOTOR_Init(bsp_motor_t *dev, UART_HandleTypeDef *uart,
                               uint8_t id, uint32_t timeout)
{
    if ((dev == NULL) || (uart == NULL) || (uart->Instance == NULL) ||
        (timeout == 0U))
    {
        return BSP_MOTOR_ERR_ARG;
    }
    if (id > BSP_MOTOR_ID_MAX)
    {
        return BSP_MOTOR_ERR_ID;
    }
    if ((uart->hdmarx == NULL) || (uart->hdmatx == NULL) ||
        (uart->hdmarx->Init.Mode != DMA_NORMAL) ||
        (uart->hdmatx->Init.Mode != DMA_NORMAL))
    {
        return BSP_MOTOR_ERR_DMA;
    }

    dev->uart = uart;
    dev->id = id;
    dev->timeout = timeout;
    dev->start_ms = 0U;
    BSP_MOTOR_ResetTransfer(dev);
    dev->test_key_ms = 0U;
    dev->test_err = BSP_MOTOR_OK;
    dev->test_state = (uint8_t)BSP_MOTOR_TEST_ENABLE;
    dev->test_next_pos = 1U;
    dev->test_key_raw = 0U;
    dev->test_key_stable = 0U;
    dev->test_armed = 0U;

    return BSP_MOTOR_OK;
}

bsp_motor_err_t BSP_MOTOR_Enable(bsp_motor_t *dev)
{
    return BSP_MOTOR_Start(dev, BSP_MOTOR_CMD_ENABLE, 0U);
}

bsp_motor_err_t BSP_MOTOR_Disable(bsp_motor_t *dev)
{
    return BSP_MOTOR_Start(dev, BSP_MOTOR_CMD_DISABLE, 0U);
}

bsp_motor_err_t BSP_MOTOR_SetAngle(bsp_motor_t *dev, float angle)
{
    float value;
    uint16_t raw;

    if ((angle != angle) || (angle < 0.0f) ||
        (angle > BSP_MOTOR_TWO_PI))
    {
        return BSP_MOTOR_ERR_RANGE;
    }

    if (angle >= BSP_MOTOR_TWO_PI)
    {
        raw = 0xFFFFU;
    }
    else
    {
        value = angle * BSP_MOTOR_U16_SCALE / BSP_MOTOR_TWO_PI;
        raw = (uint16_t)(value + 0.5f);
    }

    return BSP_MOTOR_Start(dev, BSP_MOTOR_CMD_ANGLE, raw);
}

bsp_motor_err_t BSP_MOTOR_Test(bsp_motor_t *dev, uint32_t now_ms,
                               uint8_t key_pressed)
{
    bsp_motor_err_t err;
    bsp_motor_fb_t fb;
    float angle;

    if (BSP_MOTOR_IsValid(dev) == 0U)
    {
        return BSP_MOTOR_ERR_ARG;
    }

    key_pressed = (key_pressed != 0U) ? 1U : 0U;

    switch ((bsp_motor_test_state_t)dev->test_state)
    {
        case BSP_MOTOR_TEST_ENABLE:
            err = BSP_MOTOR_Enable(dev);
            if (err == BSP_MOTOR_OK)
            {
                dev->test_state = (uint8_t)BSP_MOTOR_TEST_WAIT_ENABLE;
                return BSP_MOTOR_BUSY;
            }
            break;

        case BSP_MOTOR_TEST_WAIT_ENABLE:
            err = BSP_MOTOR_Process(dev, now_ms, &fb);
            if (err == BSP_MOTOR_BUSY)
            {
                return err;
            }
            if ((err == BSP_MOTOR_OK) && (fb.enabled != 0U))
            {
                err = BSP_MOTOR_SetAngle(dev, 0.0f);
                if (err == BSP_MOTOR_OK)
                {
                    dev->test_state = (uint8_t)BSP_MOTOR_TEST_WAIT_ZERO;
                    return BSP_MOTOR_BUSY;
                }
            }
            else if (err == BSP_MOTOR_OK)
            {
                err = BSP_MOTOR_ERR_STATE;
            }
            break;

        case BSP_MOTOR_TEST_WAIT_ZERO:
            err = BSP_MOTOR_Process(dev, now_ms, &fb);
            if (err == BSP_MOTOR_BUSY)
            {
                return err;
            }
            if ((err == BSP_MOTOR_OK) && (fb.enabled != 0U))
            {
                /* 若启动期间一直按住按键，先等待松开再允许第一次测试。 */
                dev->test_key_raw = key_pressed;
                dev->test_key_stable = key_pressed;
                dev->test_key_ms = now_ms;
                dev->test_armed = (key_pressed == 0U) ? 1U : 0U;
                dev->test_state = (uint8_t)BSP_MOTOR_TEST_READY;
                return BSP_MOTOR_OK;
            }
            if (err == BSP_MOTOR_OK)
            {
                err = BSP_MOTOR_ERR_STATE;
            }
            break;

        case BSP_MOTOR_TEST_READY:
            if (key_pressed != dev->test_key_raw)
            {
                dev->test_key_raw = key_pressed;
                dev->test_key_ms = now_ms;
            }

            if (((now_ms - dev->test_key_ms) >= BSP_MOTOR_TEST_DEBOUNCE_MS) &&
                (dev->test_key_stable != dev->test_key_raw))
            {
                dev->test_key_stable = dev->test_key_raw;
                if (dev->test_key_stable == 0U)
                {
                    dev->test_armed = 1U;
                    return BSP_MOTOR_OK;
                }
                if (dev->test_armed != 0U)
                {
                    /* 负角度按绝对角度一圈回绕后发送给电机。 */
                    angle = BSP_MOTOR_TEST_ANGLE;
                    if (dev->test_next_pos == 0U)
                    {
                        angle = BSP_MOTOR_TWO_PI - BSP_MOTOR_TEST_ANGLE;
                    }

                    err = BSP_MOTOR_SetAngle(dev, angle);
                    if (err == BSP_MOTOR_OK)
                    {
                        dev->test_armed = 0U;
                        dev->test_next_pos = (uint8_t)(dev->test_next_pos == 0U);
                        dev->test_state = (uint8_t)BSP_MOTOR_TEST_WAIT_ANGLE;
                        return BSP_MOTOR_BUSY;
                    }
                    break;
                }
            }
            return BSP_MOTOR_OK;

        case BSP_MOTOR_TEST_WAIT_ANGLE:
            err = BSP_MOTOR_Process(dev, now_ms, &fb);
            if (err == BSP_MOTOR_BUSY)
            {
                return err;
            }
            if ((err == BSP_MOTOR_OK) && (fb.enabled != 0U))
            {
                dev->test_state = (uint8_t)BSP_MOTOR_TEST_READY;
                return BSP_MOTOR_OK;
            }
            if (err == BSP_MOTOR_OK)
            {
                err = BSP_MOTOR_ERR_STATE;
            }
            break;

        case BSP_MOTOR_TEST_ERROR:
            return dev->test_err;

        default:
            err = BSP_MOTOR_ERR_STATE;
            break;
    }

    dev->test_err = err;
    dev->test_state = (uint8_t)BSP_MOTOR_TEST_ERROR;
    return err;
}

bsp_motor_err_t BSP_MOTOR_Process(bsp_motor_t *dev, uint32_t now_ms,
                                  bsp_motor_fb_t *fb)
{
    if (BSP_MOTOR_IsValid(dev) == 0U)
    {
        return BSP_MOTOR_ERR_ARG;
    }
    if (dev->active == 0U)
    {
        return BSP_MOTOR_ERR_STATE;
    }

    if (dev->dma_error != 0U)
    {
        (void)HAL_UART_DMAStop(dev->uart);
        BSP_MOTOR_ResetTransfer(dev);
        return BSP_MOTOR_ERR_DMA;
    }
    if ((now_ms - dev->start_ms) >= dev->timeout)
    {
        (void)HAL_UART_DMAStop(dev->uart);
        BSP_MOTOR_ResetTransfer(dev);
        return BSP_MOTOR_ERR_TIMEOUT;
    }
    if ((dev->tx_done == 0U) || (dev->rx_done == 0U))
    {
        return BSP_MOTOR_BUSY;
    }

    BSP_MOTOR_ResetTransfer(dev);
    if (dev->rx[0] != dev->id)
    {
        return BSP_MOTOR_ERR_ID;
    }
    if (BSP_MOTOR_Crc8(dev->rx, BSP_MOTOR_RX_LEN - 1U) != dev->rx[9])
    {
        return BSP_MOTOR_ERR_CRC;
    }

    if (fb != NULL)
    {
        fb->state = dev->rx[1];
        fb->enabled = dev->rx[1] & 0x01U;
        fb->current = BSP_MOTOR_DecodeS16(dev->rx[3], dev->rx[4],
                                          BSP_MOTOR_CURRENT_MAX);
        fb->speed = BSP_MOTOR_DecodeS16(dev->rx[5], dev->rx[6],
                                        BSP_MOTOR_SPEED_MAX);
        fb->angle = BSP_MOTOR_DecodeU16(dev->rx[7], dev->rx[8],
                                        BSP_MOTOR_TWO_PI);
    }

    return BSP_MOTOR_OK;
}

bsp_motor_err_t BSP_MOTOR_Cancel(bsp_motor_t *dev)
{
    HAL_StatusTypeDef hal;

    if (BSP_MOTOR_IsValid(dev) == 0U)
    {
        return BSP_MOTOR_ERR_ARG;
    }
    if (dev->active == 0U)
    {
        return BSP_MOTOR_OK;
    }

    hal = HAL_UART_DMAStop(dev->uart);
    BSP_MOTOR_ResetTransfer(dev);
    if (hal != HAL_OK)
    {
        return BSP_MOTOR_ERR_DMA;
    }

    return BSP_MOTOR_OK;
}

void BSP_MOTOR_OnTxComplete(bsp_motor_t *dev)
{
    if ((dev != NULL) && (dev->active != 0U))
    {
        dev->tx_done = 1U;
    }
}

void BSP_MOTOR_OnRxComplete(bsp_motor_t *dev)
{
    if ((dev != NULL) && (dev->active != 0U))
    {
        dev->rx_done = 1U;
    }
}

void BSP_MOTOR_OnError(bsp_motor_t *dev)
{
    if ((dev != NULL) && (dev->active != 0U))
    {
        dev->dma_error = 1U;
    }
}
