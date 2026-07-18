/**
 * @file pid_test.h
 * @brief PID 参数整定测试入口
 */
#ifndef PID_TEST_H
#define PID_TEST_H

/**
 * @brief 运行单轮 PID 控速整定测试
 *
 * 测试流程：
 * 1. 进入后先选择左轮或右轮。
 * 2. 仅控制被选择的轮子，另一个轮子保持停止。
 * 3. KEY1 增加目标速度，KEY2 减小目标速度，KEY3 清零并停止。
 * 4. OLED 显示实际速度、目标速度和 PID 输出。
 * 5. 串口输出实际速度、目标速度和 PID 输出，便于观察曲线。
 */
void PID_Test_Run(void);

#endif
