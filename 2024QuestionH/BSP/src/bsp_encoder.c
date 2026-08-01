/**
 * @file bsp_encoder.c
 * @brief 双路正交编码器 BSP 驱动实现
 */
#include "bsp_encoder.h"

static volatile int32_t pulse[2];
static volatile int32_t count[2];
static volatile int32_t total[2];
volatile uint8_t encoder_tick;
static volatile uint32_t time_ms;
static uint8_t time_on;

/**
 * @brief 编码器方向符号修正，用于统一前进为正
 *
 * @details
 * E1 对应 MOTOR_A，E2 对应 MOTOR_B；E2 实测方向相反，在这里翻转符号。
 */
static const int sign[2] = {
    -1,
    -1
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
 * @brief 原子锁存双路编码器累计脉冲
 */
static void encoder_latch(void)
{
    uint32_t primask;
    int32_t p0;
    int32_t p1;

    primask = __get_PRIMASK();
    __disable_irq();
    p0 = pulse[0];
    p1 = pulse[1];
    pulse[0] = 0;
    pulse[1] = 0;
    if (primask == 0U) {
        __enable_irq();
    }

    count[0] = p0 * sign[0];
    count[1] = p1 * sign[1];
    /* 在固定采样中断中累计，避免主循环因 OLED 刷新错过里程脉冲。 */
    total[0] += count[0];
    total[1] += count[1];
}

void Encoder_TimeInit(void)
{
    if (time_on != 0U) {
        return;
    }

    /* 单调时基在菜单阶段启动，后续底盘初始化不能清零。 */
    time_ms = 0U;
    time_on = 1U;
    NVIC_ClearPendingIRQ(TIMER_ENCODER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_ENCODER_TICK_INST_INT_IRQN);
}

uint32_t Encoder_GetMs(void)
{
    return time_ms;
}

void Encoder_Init(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    pulse[0] = 0;
    pulse[1] = 0;
    count[0] = 0;
    count[1] = 0;
    total[0] = 0;
    total[1] = 0;
    encoder_tick = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    Encoder_TimeInit();
}

int Encoder_Read(Encoder_ID id)
{
    return (int)count[encoder_idx(id)];
}

void Encoder_ReadTotals(int32_t *e1, int32_t *e2)
{
    uint32_t primask;
    int32_t t0;
    int32_t t1;

    primask = __get_PRIMASK();
    __disable_irq();
    t0 = total[0];
    t1 = total[1];
    if (primask == 0U) {
        __enable_irq();
    }

    if (e1 != 0) {
        *e1 = t0;
    }
    if (e2 != 0) {
        *e2 = t1;
    }
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
    encoder_latch();
    time_ms += 20U;
    encoder_tick = 1U;
}
