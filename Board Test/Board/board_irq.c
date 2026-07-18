/**
 * @file board_irq.c
 * @brief 板级中断分发入口
 */

#include "board.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "imu.h"

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
        DL_GPIO_clearInterruptStatus(GPIO_IMU_INT_PORT, GPIO_IMU_INT_PA16_PIN);
        IMU_dataReadyIrqHandler();
    }

    enc_pins = GPIO_ENCODER_E1A_PIN | GPIO_ENCODER_E1B_PIN |
               GPIO_ENCODER_E2A_PIN | GPIO_ENCODER_E2B_PIN;
    enc_status = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT, enc_pins);
    if (enc_status != 0U) {
        encoder_gpio_irq_handler(enc_status);
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, enc_pins);
    }
}

/**
 * @brief 分发编码器周期锁存定时器中断
 */
void TIMG0_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_ENCODER_TICK_INST) == DL_TIMER_IIDX_ZERO) {
        encoder_tick_irq_handler();
    }
}

/**
 * @brief 分发 UART0 中断
 */
void UART0_IRQHandler(void)
{
    BSP_Uart_IRQHandler();
}
