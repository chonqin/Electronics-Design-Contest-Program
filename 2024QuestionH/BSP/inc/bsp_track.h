/**
 * @file bsp_track.h
 * @brief 八路循迹模块 BSP 驱动接口
 */
#ifndef BSP_TRACK_H
#define BSP_TRACK_H

#include "ti_msp_dl_config.h"

/**
 * @brief 循迹通道编号
 */
typedef enum {
    TRACK_X1 = 0,
    TRACK_X2,
    TRACK_X3,
    TRACK_X4,
    TRACK_X5,
    TRACK_X6,
    TRACK_X7,
    TRACK_X8,
    TRACK_NUM
} Track_ID;

/** @brief 单路探头检测到黑线时的逻辑状态 */
#define TRACK_STATE_LINE       0U

/** @brief 单路探头未检测到黑线时的逻辑状态 */
#define TRACK_STATE_NO_LINE    1U

/** @brief 八路探头均未检测到黑线时的位图 */
#define TRACK_MASK_NO_LINE     0xFFU

/**
 * @brief 读取单路循迹状态
 * @param id 循迹通道，范围 TRACK_X1 到 TRACK_X8
 * @return 0:检测到黑线  1:未检测到黑线
 */
uint8_t Track_Read(Track_ID id);

/**
 * @brief 读取 8 路循迹模块状态位图
 * @return bit0-bit7 对应 X1-X8，位为 1 表示未检测到黑线
 */
uint8_t Track_ReadMask(void);

#endif
