/**
 * @file bsp_key.h
 * @brief 按键驱动接口
 */
#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "ti_msp_dl_config.h"

typedef enum {
    KEY_1 = 0,
    KEY_2 = 1,
    KEY_3 = 2
} Key_ID;

/**
 * @brief 按键状态枚举
 */
typedef enum {
    KEY_RELEASED = 0,  /**< 松开状态 */
    KEY_PRESSED = 1    /**< 按下状态 */
} Key_State;

/**
 * @brief 初始化按键GPIO(配置为输入+上拉)
 * @note  按键硬件为低电平有效(按下=0, 松开=1)
 */
void Key_Init(void);

/**
 * @brief 读取单个按键状态(带消抖)
 * @param key 按键选择
 * @return KEY_PRESSED:按下  KEY_RELEASED:松开
 */
Key_State Key_Read(Key_ID key);

/**
 * @brief 扫描所有按键,返回按下的按键ID
 * @return 0-2:对应按键被按下  -1:无按键按下
 */
int8_t Key_Scan(void);

/* 调试变量：最近一次读取的原始GPIO值 */
extern volatile uint32_t debug_key1_raw;
extern volatile uint32_t debug_key2_raw;
extern volatile uint32_t debug_key3_raw;

#endif
