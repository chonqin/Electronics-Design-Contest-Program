/**
 * @file bsp_encoder.c
 * @brief Dual quadrature encoder BSP driver implementation.
 */
#include "bsp_encoder.h"
#include "imu.h"

static ENCODER_RES encoder[ENCODER_NUM];

/**
 * @brief Per-encoder sign correction for unified forward direction.
 */
static const int encoder_sign[ENCODER_NUM] = {
    -1,
    1
};

/**
 * @brief Resolve an encoder selector to a valid state object.
 * @param id Encoder selector.
 * @return Pointer to encoder state.
 */
static ENCODER_RES *encoder_get_res(ENCODER_ID id)
{
    if (id >= ENCODER_NUM) {
        return &encoder[ENCODER_1];
    }

    return &encoder[id];
}

/**
 * @brief Count one quadrature encoder from edge flags and peer phase level.
 * @param res Encoder state to update.
 * @param status Enabled interrupt status for the encoder pins.
 * @param a_pin A phase GPIO pin mask.
 * @param b_pin B phase GPIO pin mask.
 */
static void encoder_count_ab(ENCODER_RES *res, uint32_t status, uint32_t a_pin, uint32_t b_pin)
{
    if ((status & a_pin) != 0U) {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, b_pin) == 0U) {
            res->temp_count--;
        } else {
            res->temp_count++;
        }
    }

    if ((status & b_pin) != 0U) {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, a_pin) == 0U) {
            res->temp_count++;
        } else {
            res->temp_count--;
        }
    }
}

/**
 * @brief Latch one encoder pulse accumulation into the public state.
 * @param res Encoder state to latch.
 * @param id Encoder selector.
 */
static void encoder_latch(ENCODER_RES *res, ENCODER_ID id)
{
    res->count = (int)(res->temp_count * encoder_sign[id]);
    res->dir = (res->count >= 0) ? FORWARD : REVERSAL;
    res->temp_count = 0;
}

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
    return encoder_get_count(ENCODER_1);
}

/**
 * @brief Get the latest encoder direction.
 * @return Current rotation direction.
 */
ENCODER_DIR get_encoder_dir(void)
{
    return encoder_get_dir(ENCODER_1);
}

int encoder_get_count(ENCODER_ID id)
{
    return encoder_get_res(id)->count;
}

ENCODER_DIR encoder_get_dir(ENCODER_ID id)
{
    return encoder_get_res(id)->dir;
}

/**
 * @brief Latch encoder pulse accumulation into the public state.
 */
void encoder_update(void)
{
    encoder_latch(&encoder[ENCODER_1], ENCODER_1);
    encoder_latch(&encoder[ENCODER_2], ENCODER_2);
}

/**
 * @brief Shared GROUP1 IRQ handler for IMU and encoder GPIO interrupts.
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

    encoder_count_ab(&encoder[ENCODER_1], enc_status,
                     GPIO_ENCODER_E1A_PIN, GPIO_ENCODER_E1B_PIN);
    encoder_count_ab(&encoder[ENCODER_2], enc_status,
                     GPIO_ENCODER_E2A_PIN, GPIO_ENCODER_E2B_PIN);

    if (enc_status != 0U) {
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT, enc_pins);
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
