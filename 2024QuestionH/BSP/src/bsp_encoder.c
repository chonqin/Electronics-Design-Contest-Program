/**
 * @file bsp_encoder.c
 * @brief 双路正交编码器 BSP 驱动实现
 */
#include "bsp_encoder.h"

static volatile long long pulse[2];
static int count[2];

/**
 * @brief 编码器方向符号修正，用于统一前进为正
 *
 * @details
 * E1 对应 MOTOR_A，E2 对应 MOTOR_B。
 */
static const int sign[2] = {
    -1,
    1
};

/**
 * @brief 将编码器编号转换成内部数组下标
 * @param id 编码器编号
 * @return 有效的内部数组下标
 */
static uint8_t encoder_idx(Encoder_ID id)
{
    if (id == ENCODER_E2) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 根据 AB 相边沿累计一个编码器脉冲
 * @param idx 编码器下标
 * @param status 编码器 GPIO 中断状态
 * @param a_pin A 相 GPIO 引脚掩码
 * @param b_pin B 相 GPIO 引脚掩码
 */
static void encoder_count_ab(uint8_t idx, uint32_t status, uint32_t a_pin, uint32_t b_pin)
{
    if ((status & a_pin) != 0U) {
        // A 相触发时读取 B 相电平，用正交关系判断增减方向。
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, b_pin) == 0U) {
            pulse[idx]--;
        } else {
            pulse[idx]++;
        }
    }

    if ((status & b_pin) != 0U) {
        // B 相触发时反查 A 相，补齐双边沿计数。
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, a_pin) == 0U) {
            pulse[idx]++;
        } else {
            pulse[idx]--;
        }
    }
}

/**
 * @brief 将单个编码器累计脉冲锁存为当前周期计数
 * @param idx 编码器下标
 */
static void encoder_latch(uint8_t idx)
{
    // 定时锁存把一个采样周期内的脉冲数转成速度反馈。
    count[idx] = (int)(pulse[idx] * sign[idx]);
    pulse[idx] = 0;
}

void Encoder_Init(void)
{
    pulse[0] = 0;
    pulse[1] = 0;
    count[0] = 0;
    count[1] = 0;

    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    NVIC_ClearPendingIRQ(TIMER_ENCODER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_ENCODER_TICK_INST_INT_IRQN);
}

int Encoder_Read(Encoder_ID id)
{
    return count[encoder_idx(id)];
}

/**
 * @brief 处理编码器 GPIO 边沿中断状态
 * @param status 编码器 GPIO 中断状态
 */
void Encoder_GpioIrqHandler(uint32_t status)
{
    // 两路编码器共用一组端口中断，这里按引脚集合拆分处理。
    encoder_count_ab(0U, status, GPIO_ENCODER_E1A_PIN, GPIO_ENCODER_E1B_PIN);
    encoder_count_ab(1U, status, GPIO_ENCODER_E2A_PIN, GPIO_ENCODER_E2B_PIN);
}

/**
 * @brief 处理编码器周期锁存定时器中断
 */
void Encoder_TickIrqHandler(void)
{
    encoder_latch(0U);
    encoder_latch(1U);
}
