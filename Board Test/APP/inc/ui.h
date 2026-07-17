/**
 * @file    ui.h
 * @brief   用户界面头文件，封装各模块的 OLED 显示与菜单逻辑
 */
#ifndef __UI_H
#define __UI_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  任务 ID                                                            */
/* ------------------------------------------------------------------ */
typedef enum {
    TASK_1 = 0,
    TASK_2 = 1,
    TASK_3 = 2,
    TASK_4 = 3,
    TASK_5 = 4,
    TASK_COUNT = 5,
    TASK_NONE  = -1   /**< 无操作（取消或无效） */
} Task_ID;

/* ------------------------------------------------------------------ */
/*  基础显示函数                                                       */
/* ------------------------------------------------------------------ */

/** @brief 初始化用户界面（OLED 正常方向显示） */
void UI_Init(void);

/** @brief 显示 OLED 静态测试界面 */
void UI_Test_OLED(void);

/**
 * @brief 刷新双编码器实时计数到 OLED
 * @param e1 编码器 1 计数
 * @param e2 编码器 2 计数
 */
void UI_Test_Encoder(int e1, int e2);

/**
 * @brief 刷新电机占空比测试值到 OLED
 * @param duty 电机 PWM 占空比命令，范围 [-4000, 4000]
 * @param e1 编码器 1 速度，单位为单次采样编码器计数
 * @param e2 编码器 2 速度，单位为单次采样编码器计数
 */
void UI_Test_Motor(int duty, int e1, int e2);

/**
 * @brief 显示 PID 轮子选择界面
 * @param left 1=左轮，0=右轮
 */
void UI_Test_PIDSelect(uint8_t left);

/**
 * @brief 刷新 PID 调参值到 OLED
 * @param wheel 轮子名称
 * @param actual 当前速度，单位为单次采样编码器计数
 * @param target 设定速度，单位为单次采样编码器计数
 * @param output PID 输出，占空比命令
 */
void UI_Test_PID(const char *wheel, int actual, int target, int output);

/** @brief 显示 IMU 校准等待界面 */
void UI_IMU_Calibrating(void);

/**
 * @brief 刷新 IMU 姿态角到 OLED
 * @param angles float[3]：角度顺序为 [0]=航向角、[1]=俯仰角、[2]=横滚角
 */
void UI_Test_IMU(float *angles);

/* ------------------------------------------------------------------ */
/*  菜单                                                               */
/* ------------------------------------------------------------------ */

/**
 * @brief 运行多级菜单，阻塞至用户确认选择
 * @return 用户确认的 Task_ID（TASK_NONE 不会返回，内部取消后重进一级菜单）
 */
Task_ID UI_Process(void);

#endif /* __UI_H */
