/**
 * @file board_irq.c
 * @brief 板级中断分发入口
 */

#include "board.h"
#include "bsp_uart.h"
#include "imu.h"

/**
 * @brief 处理编码器 GPIO 边沿中断状态
 * @param status 编码器 GPIO 中断状态
 */
void Encoder_GpioIrqHandler(uint32_t status);

/**
 * @brief 处理编码器周期锁存定时器中断
 */
void Encoder_TickIrqHandler(void);

/**
 * @brief 分发 GROUP1 GPIO 中断
 *
 * @details 当前 GROUP1 同时承载 IMU 数据就绪中断和双路编码器边沿中断。
 */
void GROUP1_IRQHandler(void)
{
    uint32_t imu_status;
    uint32_t enc_status;
    uint32_t enc_pins;

    imu_status = DL_GPIO_getEnabledInterruptStatus(GPIO_IMU_INT_PORT,
                                                   GPIO_IMU_INT_PA16_PIN);
    if ((imu_status & GPIO_IMU_INT_PA16_PIN) != 0U) {
        // IMU 数据就绪优先清中断标志，再进入姿态采样流程。
        DL_GPIO_clearInterruptStatus(GPIO_IMU_INT_PORT, GPIO_IMU_INT_PA16_PIN);
        IMU_dataReadyIrqHandler();
    }

    enc_pins = GPIO_ENCODER_E1A_PIN | GPIO_ENCODER_E1B_PIN |
               GPIO_ENCODER_E2A_PIN | GPIO_ENCODER_E2B_PIN;
    enc_status = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT, enc_pins);
    if (enc_status != 0U) {
        // 编码器多路边沿共用一组中断，交给编码器模块细分处理。
        Encoder_GpioIrqHandler(enc_status);
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, enc_pins);
    }
}

/**
 * @brief 分发编码器周期锁存定时器中断
 */
void TIMG0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_ENCODER_TICK_INST) == DL_TIMERG_IIDX_ZERO) {
        // 周期节拍到来时锁存脉冲数，生成新的速度反馈值。
        Encoder_TickIrqHandler();
    }
}

/**
 * @brief 分发 UART0 中断
 */
void UART0_IRQHandler(void)
{
    BSP_Uart_IRQHandler();
}
