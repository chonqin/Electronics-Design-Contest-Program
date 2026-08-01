/**
 * @file task.h
 * @brief 任务 1 至任务 6 接口
 */
#ifndef TASK_H
#define TASK_H

#include <stdint.h>

/** @brief 运行空任务 1，并显示图传测试提示。 */
void task1_run(void);

/**
 * @brief 运行任务 2 循迹逻辑
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 */
void task2_run(uint32_t start_ms);

/** @brief 运行任务 3 的 PB0 高电平门控计时逻辑。 */
void task3_run(void);

/**
 * @brief 运行任务 4 循迹逻辑
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 */
void task4_run(uint32_t start_ms);

/**
 * @brief 运行任务 5 循迹逻辑
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 */
void task5_run(uint32_t start_ms);

/**
 * @brief 运行任务 6 循迹逻辑
 * @param start_ms 确认按键被识别时的时间戳，单位为 ms
 */
void task6_run(uint32_t start_ms);

#endif
