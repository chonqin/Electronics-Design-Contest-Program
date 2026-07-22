/**
 * @file    bsp_key.c
 * @brief   按键驱动实现（低电平有效，带消抖）
 */
#include "bsp_key.h"
#include "board.h"

volatile uint32_t debug_key1_raw = 0;
volatile uint32_t debug_key2_raw = 0;
volatile uint32_t debug_key3_raw = 0;

/**
 * @brief 初始化按键 GPIO（输入 + 上拉）
 */
void Key_Init(void)
{
    DL_GPIO_initDigitalInputFeatures(GPIO_KEY_PIN_18_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_KEY_PIN_13_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_KEY_PIN_17_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/**
 * @brief 读取单个按键状态（10ms 消抖）
 * @param key  按键 ID
 * @return KEY_PRESSED / KEY_RELEASED
 */
Key_State Key_Read(Key_ID key)
{
    uint32_t raw1, raw2;

    switch (key) {
        case KEY_1:
            raw1 = DL_GPIO_readPins(GPIO_KEY_PIN_18_PORT, GPIO_KEY_PIN_18_PIN);
            debug_key1_raw = raw1;
            if (raw1 != 0) return KEY_RELEASED;
            delay_ms(10);
            raw2 = DL_GPIO_readPins(GPIO_KEY_PIN_18_PORT, GPIO_KEY_PIN_18_PIN);
            return (raw2 == 0) ? KEY_PRESSED : KEY_RELEASED;

        case KEY_2:
            raw1 = DL_GPIO_readPins(GPIO_KEY_PIN_13_PORT, GPIO_KEY_PIN_13_PIN);
            debug_key2_raw = raw1;
            if (raw1 != 0) return KEY_RELEASED;
            delay_ms(10);
            raw2 = DL_GPIO_readPins(GPIO_KEY_PIN_13_PORT, GPIO_KEY_PIN_13_PIN);
            return (raw2 == 0) ? KEY_PRESSED : KEY_RELEASED;

        case KEY_3:
            raw1 = DL_GPIO_readPins(GPIO_KEY_PIN_17_PORT, GPIO_KEY_PIN_17_PIN);
            debug_key3_raw = raw1;
            if (raw1 != 0) return KEY_RELEASED;
            delay_ms(10);
            raw2 = DL_GPIO_readPins(GPIO_KEY_PIN_17_PORT, GPIO_KEY_PIN_17_PIN);
            return (raw2 == 0) ? KEY_PRESSED : KEY_RELEASED;

        default:
            return KEY_RELEASED;
    }
}

/**
 * @brief 扫描所有按键，返回第一个按下的按键 ID
 * @return 0-2 对应按键ID，-1 表示无按键
 */
int8_t Key_Scan(void)
{
    for (int8_t k = 0; k < 3; k++) {
        if (Key_Read((Key_ID)k) == KEY_PRESSED)
            return k;
    }
    return -1;
}
