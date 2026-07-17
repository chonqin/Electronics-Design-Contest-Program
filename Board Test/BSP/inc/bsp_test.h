/**
 * @file bsp_test.h
 * @brief BSP模块测试例程接口
 */
#ifndef BSP_TEST_H
#define BSP_TEST_H

#include "ti_msp_dl_config.h"

/**
 * @brief 编码器电机综合测试
 *
 * 测试流程：
 *   1. 电机A正转2s → 检测编码器1是否有脉冲 → LED1亮表示正常
 *   2. 电机A反转2s → 检测编码器1方向是否翻转 → LED2亮表示正常
 *   3. 电机B重复以上流程
 *   4. 全部通过：LED1 LED2同时闪烁
 *   5. 任一失败：对应LED常灭
 */

/**
 * @brief 带 OLED 编码器反馈的电机 PWM 占空比测试
 * @details KEY1 增加占空比，KEY2 减少占空比，KEY3 清零占空比
 */
void BSP_Test_Motor(void);

/**
 * @brief OLED显示测试
 *
 * 测试流程：
 *   1. 初始化OLED
 *   2. 显示字符串验证文字输出
 *   3. 显示数字验证数值显示
 *   4. 画线、画圆验证图形绘制
 *   5. 循环刷新计数器验证持续通信
 */
void BSP_Test_OLED(void);

/**
 * @brief 按键+LED测试
 *
 * 测试流程：
 *   1. 初始化按键GPIO和LED
 *   2. 循环扫描3个按键
 *   3. 任意按键按下 → LED1和LED2点亮
 *   4. 按键释放 → LED1和LED2熄灭
 */
void BSP_Test_KEY(void);

/**
 * @brief 编码器独立测试（电机驱动，LED指示结果）
 *
 * 测试流程：
 *   1. 电机A低速正转1s → 检测编码器1是否有正向脉冲
 *   2. 电机A低速反转1s → 检测编码器1是否有反向脉冲
 *   3. 电机B低速正转1s → 检测编码器2是否有正向脉冲
 *   4. 电机B低速反转1s → 检测编码器2是否有反向脉冲
 *   5. 结果：编码器1正常→LED1常亮，编码器2正常→LED2常亮
 *            全部通过→双LED循环闪烁，任一失败→对应LED快闪报错
 */
void BSP_Test_Encoder(void);

/**
 * @brief 带 OLED 反馈和按键目标调节的 PID 控速测试
 *
 * 测试流程：
 *   1. 先选择左轮或右轮
 *   2. 仅控制所选轮子，另一轮保持停止
 *   3. KEY1/KEY2 调整设定速度，KEY3 清零并停止
 *   4. 串口输出实际速度、设定速度和 PID 输出
 *   5. OLED 显示轮子名、实际速度、设定速度和 PID 输出
 */
void BSP_Test_PID(void);

/**
 * @brief IMU三轴数据测试
 *
 * 测试流程：
 *   1. 初始化ICM42688与OLED
 *   2. OLED实时显示原始六轴数据(陀螺仪角速度dps + 加速度g)
 *   3. 串口持续输出姿态角 俯仰角 / 横滚角 / 航向角
 *   4. 初始化失败时OLED与串口均报错提示
 */
void BSP_Test_IMU(void);

#endif
