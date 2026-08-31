/**
 * @file debug.h
 * @brief 底盘 UART 调试接口
 */
#ifndef DEBUG_H
#define DEBUG_H

#include "car.h"
#include <stdint.h>

/**
 * @brief 更新 VOFA+ 中的任务 1 用时通道
 * @param ms 任务 1 用时，单位为 ms
 */
void Debug_SetTask1Time(uint32_t ms);

/**
 * @brief 输出一帧底盘 UART 调试数据
 */
void Debug_Output(void);

/**
 * @brief 消费 UART0 RX 环形缓冲区中的在线调参命令
 * @note 支持 TKP/TKI/TKD/YKP/YKI/YKD，使用 = 设置、+ 增加、- 减少，
 *       例如 TKP=0.18、TKP+0.01、TKP-0.01；STOP 用于停车。
 *       每条命令必须以 \n 或 \r\n 结束。
 */
void Debug_CommandPoll(void);

#endif
