/**
 * @file bsp_encoder.h
 * @brief 双路正交编码器 BSP 驱动接口
 */
#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#include "ti_msp_dl_config.h"

/**
 * @brief 编码器编号
 *
 * @details
 * E1 对应 MOTOR_A，E2 对应 MOTOR_B。
 */
typedef enum {
    ENCODER_E1 = 0,
    ENCODER_E2 = 1
} Encoder_ID;

/**
 * @brief 初始化双路编码器中断和内部计数状态
 */
void Encoder_Init(void);

/**
 * @brief 读取单个编码器的最新锁存计数
 * @param id 编码器编号，取值为 ENCODER_E1 或 ENCODER_E2
 * @return 当前控制周期内锁存的编码器计数
 */
int Encoder_Read(Encoder_ID id);

#endif
