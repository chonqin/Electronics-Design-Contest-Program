/**
 * @file bsp_encoder.c
 * @brief 双路正交编码器 BSP 驱动实现
 */
#include "bsp_encoder.h"
#include "imu.h"

static ENCODER_RES encoder[ENCODER_NUM];

/**
 * @brief 各编码器方向符号修正，用于统一正转方向
 */
static const int encoder_sign[ENCODER_NUM] = {
    -1,
    1
};

/**
 * @brief 将编码器选择映射到有效状态对象
 * @param id 编码器选择
 * @return 编码器状态指针
 */
static ENCODER_RES *encoder_get_res(ENCODER_ID id)
{
    if (id >= ENCODER_NUM) {
        return &encoder[ENCODER_1];
    }

    return &encoder[id];
}

/**
 * @brief 根据边沿标志和另一相电平计算一个正交编码器计数
 * @param res 要更新的编码器状态
 * @param status 编码器引脚的使能中断状态
 * @param a_pin A 相 GPIO 引脚掩码
 * @param b_pin B 相 GPIO 引脚掩码
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
 * @brief 将单个编码器的脉冲累计值锁存到公共状态
 * @param res 要锁存的编码器状态
 * @param id 编码器选择
 */
static void encoder_latch(ENCODER_RES *res, ENCODER_ID id)
{
    res->count = (int)(res->temp_count * encoder_sign[id]);
    res->dir = (res->count >= 0) ? FORWARD : REVERSAL;
    res->temp_count = 0;
}

/**
 * @brief 初始化编码器 GPIO 和周期性锁存定时器中断
 */
void encoder_init(void)
{
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    NVIC_ClearPendingIRQ(TIMER_ENCODER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_ENCODER_TICK_INST_INT_IRQN);
}

/**
 * @brief 获取最新锁存的编码器计数
 * @return 编码器计数
 */
int get_encoder_count(void)
{
    return encoder_get_count(ENCODER_1);
}

/**
 * @brief 获取最新编码器方向
 * @return 当前旋转方向
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
 * @brief 将编码器脉冲累计值锁存到公共状态
 */
void encoder_update(void)
{
    encoder_latch(&encoder[ENCODER_1], ENCODER_1);
    encoder_latch(&encoder[ENCODER_2], ENCODER_2);
}

/**
 * @brief IMU 和编码器 GPIO 中断共用的 GROUP1 中断处理函数
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
 * @brief 周期性锁存编码器脉冲累计值
 */
void TIMG0_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_ENCODER_TICK_INST) == DL_TIMER_IIDX_ZERO) {
        encoder_update();
    }
}
