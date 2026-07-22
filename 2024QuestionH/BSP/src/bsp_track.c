/**
 * @file bsp_track.c
 * @brief 八路循迹模块 BSP 驱动实现（低电平有效）
 */
#include "bsp_track.h"

/**
 * @brief 读取单路循迹状态
 * @param id 循迹通道，范围 TRACK_X1 到 TRACK_X8
 * @return 1:检测到黑线  0:未检测到黑线
 */
uint8_t Track_Read(Track_ID id)
{
    uint32_t raw = 1U;

    switch (id) {
        case TRACK_X1:
            raw = DL_GPIO_readPins(GPIO_TRACk_X1_PORT, GPIO_TRACk_X1_PIN);
            break;

        case TRACK_X2:
            raw = DL_GPIO_readPins(GPIO_TRACk_X2_PORT, GPIO_TRACk_X2_PIN);
            break;

        case TRACK_X3:
            raw = DL_GPIO_readPins(GPIO_TRACk_X3_PORT, GPIO_TRACk_X3_PIN);
            break;

        case TRACK_X4:
            raw = DL_GPIO_readPins(GPIO_TRACk_X4_PORT, GPIO_TRACk_X4_PIN);
            break;

        case TRACK_X5:
            raw = DL_GPIO_readPins(GPIO_TRACk_X5_PORT, GPIO_TRACk_X5_PIN);
            break;

        case TRACK_X6:
            raw = DL_GPIO_readPins(GPIO_TRACk_X6_PORT, GPIO_TRACk_X6_PIN);
            break;

        case TRACK_X7:
            raw = DL_GPIO_readPins(GPIO_TRACk_X7_PORT, GPIO_TRACk_X7_PIN);
            break;

        case TRACK_X8:
            raw = DL_GPIO_readPins(GPIO_TRACk_X8_PORT, GPIO_TRACk_X8_PIN);
            break;

        default:
            break;
    }

    /* 循迹模块检测到黑线时输出低电平。 */
    if (raw == 0) {
        return 1;
    }

    return 0;
}

/**
 * @brief 读取 8 路循迹黑线状态位图
 * @return bit0-bit7 对应 X1-X8，位为 1 表示该路检测到黑线
 */
uint8_t Track_ReadMask(void)
{
    uint8_t mask = 0;

    mask |= (uint8_t)(Track_Read(TRACK_X1) << 0);
    mask |= (uint8_t)(Track_Read(TRACK_X2) << 1);
    mask |= (uint8_t)(Track_Read(TRACK_X3) << 2);
    mask |= (uint8_t)(Track_Read(TRACK_X4) << 3);
    mask |= (uint8_t)(Track_Read(TRACK_X5) << 4);
    mask |= (uint8_t)(Track_Read(TRACK_X6) << 5);
    mask |= (uint8_t)(Track_Read(TRACK_X7) << 6);
    mask |= (uint8_t)(Track_Read(TRACK_X8) << 7);

    return mask;
}
