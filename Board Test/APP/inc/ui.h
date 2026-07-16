/**
 * @file    ui.h
 * @brief   用户界面头文件，封装各模块的 OLED 显示与菜单逻辑
 */
#ifndef __UI_H
#define __UI_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  任务 ID                                                             */
/* ------------------------------------------------------------------ */
typedef enum {
    TASK_1 = 0,
    TASK_2 = 1,
    TASK_3 = 2,
    TASK_4 = 3,
    TASK_COUNT = 4,
    TASK_NONE  = -1   /**< 无操作（取消或无效） */
} Task_ID;

/* ------------------------------------------------------------------ */
/*  基础显示函数                                                        */
/* ------------------------------------------------------------------ */

/** @brief 初始化用户界面（OLED 正常方向显示） */
void UI_Init(void);

/** @brief 显示编码器测试静态布局 */
void UI_Test_OLED(void);

/**
 * @brief 刷新编码器实时数据到 OLED
 * @param count  编码器计数值
 * @param val_i  全局脉冲计数 i
 * @param val_j  全局脉冲计数 j
 */
void UI_Test_Encoder(uint32_t count);

/** @brief 显示 IMU 校准等待界面 */
void UI_IMU_Calibrating(void);

/**
 * @brief 刷新 IMU 姿态角到 OLED
 * @param angles  float[3]：[0]=Yaw, [1]=Pitch, [2]=Roll
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
