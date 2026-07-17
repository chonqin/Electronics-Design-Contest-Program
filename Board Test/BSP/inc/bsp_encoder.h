/**
 * @file bsp_encoder.h
 * @brief 双路正交编码器 BSP 驱动接口
 */
#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#include "ti_msp_dl_config.h"

typedef enum {
    FORWARD,
    REVERSAL
} ENCODER_DIR;

typedef enum {
    ENCODER_1 = 0,
    ENCODER_2,
    ENCODER_NUM
} ENCODER_ID;

typedef struct {
    volatile long long temp_count;
    int count;
    ENCODER_DIR dir;
} ENCODER_RES;

/**
 * @brief 初始化编码器 GPIO 和周期性锁存定时器中断
 */
void encoder_init(void);

/**
 * @brief 获取编码器 1 最新锁存计数
 * @return 编码器 1 计数
 */
int get_encoder_count(void);

/**
 * @brief 获取编码器 1 最新方向
 * @return 编码器 1 方向
 */
ENCODER_DIR get_encoder_dir(void);

/**
 * @brief 获取指定编码器最新锁存计数
 * @param id 编码器选择
 * @return 指定编码器计数
 */
int encoder_get_count(ENCODER_ID id);

/**
 * @brief 获取指定编码器最新方向
 * @param id 编码器选择
 * @return 指定编码器方向
 */
ENCODER_DIR encoder_get_dir(ENCODER_ID id);

/**
 * @brief 将所有编码器脉冲累计值锁存到公共状态
 */
void encoder_update(void);

#endif
