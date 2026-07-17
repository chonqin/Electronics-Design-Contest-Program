/**
 * @file    test_icm42688.c
 * @brief   ICM42688P 驱动测试辅助函数
 */

#include "ti_msp_dl_config.h"
#include "bsp_icm42688.h"
#include "bsp_test.h"
#include "imu.h"
#include <stdio.h>

/** @brief 由 imu.c 导出的 IMU 采样节拍 */
extern uint32_t nowtime;

/**
 * @brief 测试设备 ID 通信
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
 * @brief 测试换算后的传感器数据读取
 */
void test_read_raw_data(void)
{
    icm42688_real_data_t acc;
    icm42688_real_data_t gyro;
    int16_t temp;

    printf("\r\n=== Test 2: Read Raw Data ===\r\n");

    bsp_IcmGet_Temperature(&temp);
    printf("Temperature: %d C\r\n", temp);

    for (int i = 0; i < 5; i++) {
        bsp_IcmGetRawData(&acc, &gyro);

        printf("Acc(g):  X=%.3f, Y=%.3f, Z=%.3f\r\n", acc.x, acc.y, acc.z);
        printf("Gyro(dps): X=%.2f, Y=%.2f, Z=%.2f\r\n", gyro.x, gyro.y, gyro.z);

        delay_ms(500);
    }
}

/**
 * @brief 通过数据就绪中断采样测试 AHRS 结果
 * @note 保持 INT1 路由到 PA16，并在 GROUP1_IRQHandler() 中调用 IMU_dataReadyIrqHandler()
 */
void test_attitude(void)
{
    float angles[3];

    printf("\r\n=== Test 3: Attitude Estimation ===\r\n");
    printf("NOTE: This test requires the ICM42688P data-ready interrupt.\r\n");
    printf("Please route INT1 to PA16 and call IMU_dataReadyIrqHandler() in GROUP1_IRQHandler().\r\n\r\n");

    IMU_init();

    printf("Calibrating gyro offset... Keep the sensor still!\r\n");
    delay_ms(2000);
    printf("Calibration done!\r\n\r\n");

    printf("Reading attitude angles...\r\n");
    for (int i = 0; i < 20; i++) {
        IMU_getYawPitchRoll(angles);

        printf("Yaw=%.2f, Pitch=%.2f, Roll=%.2f\r\n",
               angles[0], angles[1], angles[2]);

        delay_ms(100);
    }
}

/**
 * @brief 独立 ICM42688P 测试入口
 */
void icm42688_test_main(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("  ICM42688 Driver Test for MSPM0G3507  \r\n");
    printf("========================================\r\n");

    test_read_id();
    delay_ms(1000);

    test_read_raw_data();
    delay_ms(1000);

    /* 当 PA16 数据就绪中断路径启用后再取消注释 */
    /* test_attitude(); */

    printf("\r\n========================================\r\n");
    printf("  All tests completed!  \r\n");
    printf("========================================\r\n");
}

/**
 * @brief GPIO 数据就绪中断示例入口
 */
void IMU_GPIO_ISR_Example(void)
{
    IMU_dataReadyIrqHandler();
}
