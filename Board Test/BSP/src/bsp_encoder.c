#include "bsp_encoder.h"
#include "imu.h"

static ENCODER_RES motor_encoder;

/**
 * @brief Initialize encoder GPIO and periodic latch timer interrupts.
 */
void encoder_init(void)
{
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    NVIC_ClearPendingIRQ(TIMER_ENCODER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_ENCODER_TICK_INST_INT_IRQN);
}

/**
 * @brief Get the latest latched encoder count.
 * @return Encoder count.
 */
int get_encoder_count(void)
{
    return motor_encoder.count;
}

/**
 * @brief Get the latest encoder direction.
 * @return Current rotation direction.
 */
ENCODER_DIR get_encoder_dir(void)
{
    return motor_encoder.dir;
}

/**
 * @brief Latch encoder pulse accumulation into the public state.
 */
void encoder_update(void)
{
    motor_encoder.count = motor_encoder.temp_count;
    motor_encoder.dir = (motor_encoder.count >= 0) ? FORWARD : REVERSAL;
    motor_encoder.temp_count = 0;
}

/**
 * @brief Shared GROUP1 IRQ handler for IMU and encoder GPIO interrupts.
 */
void GROUP1_IRQHandler(void)
{
    uint32_t imu_status;
    uint32_t enc_status;

    imu_status = DL_GPIO_getEnabledInterruptStatus(GPIO_IMU_INT_PORT,
                                                   GPIO_IMU_INT_PA16_PIN);
    if ((imu_status & GPIO_IMU_INT_PA16_PIN) != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_IMU_INT_PORT, GPIO_IMU_INT_PA16_PIN);
        IMU_dataReadyIrqHandler();
    }

    enc_status = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT,
                                                   GPIO_ENCODER_E1A_PIN | GPIO_ENCODER_E1B_PIN);
    if ((enc_status & GPIO_ENCODER_E1A_PIN) != 0U) {
        if (!DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_E1B_PIN)) {
            motor_encoder.temp_count--;
        } else {
            motor_encoder.temp_count++;
        }
    } else if ((enc_status & GPIO_ENCODER_E1B_PIN) != 0U) {
        if (!DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_E1A_PIN)) {
            motor_encoder.temp_count++;
        } else {
            motor_encoder.temp_count--;
        }
    }

    if (enc_status != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT,
                                     GPIO_ENCODER_E1A_PIN | GPIO_ENCODER_E1B_PIN);
    }
}

/**
 * @brief Periodically latch encoder pulse accumulation.
 */
void TIMG0_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_ENCODER_TICK_INST) == DL_TIMER_IIDX_ZERO) {
        encoder_update();
    }
}
