/**
 * @file    test_icm42688.c
 * @brief   ICM42688驱动测试示例
 * @note    测试步骤:
 *          1. 初始化ICM42688
 *          2. 读取设备ID验证通信
 *          3. 循环读取加速度、陀螺仪、姿态角并打印
 */

#include "ti_msp_dl_config.h"
#include "bsp_icm42688.h"
#include "bsp_test.h"
#include "imu.h"
#include <stdio.h>

/* 外部变量声明 */
extern uint32_t nowtime; // 时间计数器(需要在定时中断中递增)

/**
 * @brief  测试函数1: 读取设备ID
 */
void test_read_id(void)
{
    printf("\r\n=== Test 1: Read Device ID ===\r\n");

    if (ICM_ReadID()) {
        printf("[PASS] Device ID = 0x47\r\n");
    } else {
        printf("[FAIL] Cannot read device ID!\r\n");
    }
}

/**
 * @brief  测试函数2: 读取原始数据
 */
void test_read_raw_data(void)
{
    icm42688_real_data_t acc, gyro;
    int16_t temp;

    printf("\r\n=== Test 2: Read Raw Data ===\r\n");

    /* 读取温度 */
    bsp_IcmGet_Temperature(&temp);
    printf("Temperature: %d C\r\n", temp);

    /* 读取加速度和陀螺仪 */
    for (int i = 0; i < 5; i++) {
        bsp_IcmGetRawData(&acc, &gyro);

        printf("Acc(g):  X=%.3f, Y=%.3f, Z=%.3f\r\n", acc.x, acc.y, acc.z);
        printf("Gyro(dps): X=%.2f, Y=%.2f, Z=%.2f\r\n", gyro.x, gyro.y, gyro.z);

        delay_ms(500);
    }
}

/**
 * @brief  测试函数3: 姿态解算(需要配置定时器)
 * @note   需要先配置定时器,在中断中调用IMU_sample()并递增nowtime
 */
void test_attitude(void)
{
    float angles[3];

    printf("\r\n=== Test 3: Attitude Estimation ===\r\n");
    printf("NOTE: This test requires a timer interrupt!\r\n");
    printf("Please configure a 5ms timer and call IMU_sample() in ISR.\r\n\r\n");

    /* 初始化IMU姿态解算 */
    IMU_init();

    /* 等待陀螺仪零偏校准(静止约10秒) */
    printf("Calibrating gyro offset... Keep the sensor still!\r\n");
    delay_ms(10000);
    printf("Calibration done!\r\n\r\n");

    /* 循环读取姿态角 */
    printf("Reading attitude angles...\r\n");
    for (int i = 0; i < 20; i++) {
        IMU_getYawPitchRoll(angles);

        printf("Yaw=%.2f, Pitch=%.2f, Roll=%.2f\r\n",
               angles[0], angles[1], angles[2]);

        delay_ms(100);
    }
}

/**
 * @brief  主测试入口
 */
void icm42688_test_main(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("  ICM42688 Driver Test for MSPM0G3507  \r\n");
    printf("========================================\r\n");

    /* 测试1: 读取设备ID */
    test_read_id();
    delay_ms(1000);

    /* 测试2: 读取原始数据 */
    test_read_raw_data();
    delay_ms(1000);

    /* 测试3: 姿态解算(可选,需要定时器支持) */
    // test_attitude(); // 取消注释以测试姿态解算

    printf("\r\n========================================\r\n");
    printf("  All tests completed!  \r\n");
    printf("========================================\r\n");
}

/**
 * @brief  定时器中断服务函数示例
 * @note   需要在SysConfig中配置一个5ms定时器,并在中断中调用此函数
 */
void TIMER_ISR_Example(void)
{
    /* 递增时间计数器 */
    nowtime++;

    /* 采样IMU数据 */
    IMU_sample();
}
