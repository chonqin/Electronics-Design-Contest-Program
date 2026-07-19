/**
 * @file test.h
 * @brief 硬件联调测试入口
 */
#ifndef TEST_H
#define TEST_H

#include "ti_msp_dl_config.h"

/**
 * @brief 带 OLED 和编码器反馈的电机 PWM 占空比测试
 *
 * @details KEY1 增加占空比，KEY2 减少占空比，KEY3 清零占空比。
 */
void Test_Motor(void);

/**
 * @brief OLED 显示测试
 */
void Test_OLED(void);

/**
 * @brief 按键和 LED 联调测试
 */
void Test_KEY(void);

/**
 * @brief 编码器和电机联调测试
 */
void Test_Encoder(void);

/**
 * @brief IMU 姿态数据显示测试
 */
void Test_IMU(void);

/**
 * @brief UART0 接收回发测试入口
 */
void Test_UartReceive(void);

/**
 * @brief 循迹模块 OLED 和串口显示测试
 */
void Test_Track(void);

#endif
