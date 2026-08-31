/**
 * @file bsp_motor.h
 * @brief QD4310 电机绝对角度 DMA 驱动接口。
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/** @brief QD4310 电机事务执行结果。 */
typedef enum
{
    BSP_MOTOR_OK = 0,
    BSP_MOTOR_BUSY,
    BSP_MOTOR_ERR_ARG,
    BSP_MOTOR_ERR_RANGE,
    BSP_MOTOR_ERR_STATE,
    BSP_MOTOR_ERR_DMA,
    BSP_MOTOR_ERR_TX,
    BSP_MOTOR_ERR_RX,
    BSP_MOTOR_ERR_TIMEOUT,
    BSP_MOTOR_ERR_ID,
    BSP_MOTOR_ERR_CRC
} bsp_motor_err_t;

/** @brief QD4310 电机反馈数据的解析结果。 */
typedef struct
{
    uint8_t state;
    uint8_t enabled;
    float current;
    float speed;
    float angle;
} bsp_motor_fb_t;

/** @brief QD4310 电机 DMA 事务上下文。 */
typedef struct
{
    UART_HandleTypeDef *uart;
    uint32_t timeout;
    uint32_t start_ms;
    uint8_t id;
    uint8_t tx[5];
    uint8_t rx[10];
    volatile uint8_t active;
    volatile uint8_t tx_done;
    volatile uint8_t rx_done;
    volatile uint8_t dma_error;
    uint32_t test_key_ms;
    bsp_motor_err_t test_err;
    uint8_t test_state;
    uint8_t test_next_pos;
    uint8_t test_key_raw;
    uint8_t test_key_stable;
    uint8_t test_armed;
} bsp_motor_t;

/**
 * @brief 初始化 QD4310 电机 DMA 上下文，不改变 UART 或 DMA 配置。
 * @param dev 待初始化的电机上下文。
 * @param uart 已配置 Normal 模式 TX/RX DMA 的 HAL UART 句柄。
 * @param id 电机 ID，有效范围为 0x00～0x0F。
 * @param timeout 单次完整事务超时时间，单位为毫秒，必须大于 0。
 * @return 成功返回 BSP_MOTOR_OK，否则返回参数、ID 或 DMA 配置错误。
 */
bsp_motor_err_t BSP_MOTOR_Init(bsp_motor_t *dev, UART_HandleTypeDef *uart,
                               uint8_t id, uint32_t timeout);

/**
 * @brief 异步启动电机使能事务。
 * @param dev 已初始化的电机上下文。
 * @return 成功启动返回 BSP_MOTOR_OK，事务未结束返回 BSP_MOTOR_BUSY。
 */
bsp_motor_err_t BSP_MOTOR_Enable(bsp_motor_t *dev);

/**
 * @brief 异步启动电机失能事务。
 * @param dev 已初始化的电机上下文。
 * @return 成功启动返回 BSP_MOTOR_OK，事务未结束返回 BSP_MOTOR_BUSY。
 */
bsp_motor_err_t BSP_MOTOR_Disable(bsp_motor_t *dev);

/**
 * @brief 异步启动绝对角度控制事务。
 * @param dev 已初始化的电机上下文。
 * @param angle 绝对角度，单位为 rad，有效范围为 0～2*pi。
 * @return 成功启动返回 BSP_MOTOR_OK，否则返回忙、参数或范围错误。
 */
bsp_motor_err_t BSP_MOTOR_SetAngle(bsp_motor_t *dev, float angle);

/**
 * @brief 在主循环中执行电机零点与正负角度按键测试。
 * @param dev 已初始化的电机上下文。
 * @param now_ms 当前 HAL tick，单位为毫秒。
 * @param key_pressed 按键按下状态，按下传入 1，松开传入 0。
 * @return 空闲就绪返回 BSP_MOTOR_OK，事务执行中返回 BSP_MOTOR_BUSY，
 *         发生通信或反馈错误时返回具体错误。
 * @note 首次调用后自动使能并进入 0 rad；此后每次有效按下依次切换
 *       +0.4 rad 和 -0.4 rad，函数内部包含 20 ms 消抖。
 */
bsp_motor_err_t BSP_MOTOR_Test(bsp_motor_t *dev, uint32_t now_ms,
                               uint8_t key_pressed);

/**
 * @brief 在主循环中推进电机事务并解析已完成的反馈。
 * @param dev 已初始化且存在活动事务的电机上下文。
 * @param now_ms 当前 HAL tick，单位为毫秒。
 * @param fb 可选的反馈保存地址，允许为 NULL。
 * @return 未完成返回 BSP_MOTOR_BUSY，完成返回 BSP_MOTOR_OK 或具体错误。
 */
bsp_motor_err_t BSP_MOTOR_Process(bsp_motor_t *dev, uint32_t now_ms,
                                  bsp_motor_fb_t *fb);

/**
 * @brief 停止当前 DMA 事务并清除事务标志。
 * @param dev 已初始化的电机上下文。
 * @return 成功返回 BSP_MOTOR_OK，否则返回参数或 DMA 错误。
 */
bsp_motor_err_t BSP_MOTOR_Cancel(bsp_motor_t *dev);

/**
 * @brief 通知驱动 USART 发送 DMA 已完成。
 * @param dev 电机上下文。
 */
void BSP_MOTOR_OnTxComplete(bsp_motor_t *dev);

/**
 * @brief 通知驱动 USART 接收 DMA 已完成。
 * @param dev 电机上下文。
 */
void BSP_MOTOR_OnRxComplete(bsp_motor_t *dev);

/**
 * @brief 通知驱动 USART 或 DMA 发生错误。
 * @param dev 电机上下文。
 */
void BSP_MOTOR_OnError(bsp_motor_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
