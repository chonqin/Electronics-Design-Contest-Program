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
 * @brief 20 ms chassis control tick set after encoder samples are latched.
 */
extern volatile uint8_t encoder_tick;

/**
 * @brief 启动基于编码器周期定时器的单调时间基准
 * @note 计时分辨率为 20 ms，重复调用不会清零累计时间。
 */
void Encoder_TimeInit(void);

/**
 * @brief 获取系统启动后的单调时间
 * @return 当前累计时间，单位为 ms
 */
uint32_t Encoder_GetMs(void);

/**
 * @brief 初始化双路编码器中断和内部计数状态
 */
void Encoder_Init(void);

/**
 * @brief 读取单个编码器的最新 20 ms 锁存计数
 * @param id 编码器编号，取值为 ENCODER_E1 或 ENCODER_E2
 * @return 当前控制周期内锁存的编码器计数
 */
int Encoder_Read(Encoder_ID id);

/**
 * @brief 原子读取双路编码器自初始化以来的累计计数
 * @param e1 输出 E1 累计计数，允许为空指针
 * @param e2 输出 E2 累计计数，允许为空指针
 * @note 累计值已经过方向修正，小车前进时应同时增加。
 */
void Encoder_ReadTotals(int32_t *e1, int32_t *e2);

#endif
