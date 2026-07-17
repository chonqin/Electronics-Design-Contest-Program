/**
 * @file    bsp_icm42688.h
 * @brief   ICM42688P SPI 驱动接口
 */

#ifndef __BSP_ICM42688_H
#define __BSP_ICM42688_H

#include <stdint.h>
#include "board.h"

/* 设备 ID */
#define ICM_DEVICE_ID                      0x47

/* 0 号寄存器组 */
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_TMST_FSYNCH               0x2B
#define ICM42688_TMST_FSYNCL               0x2C
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_FIFO_COUNTH               0x2E
#define ICM42688_FIFO_COUNTL               0x2F
#define ICM42688_FIFO_DATA                 0x30
#define ICM42688_APEX_DATA0                0x31
#define ICM42688_APEX_DATA1                0x32
#define ICM42688_APEX_DATA2                0x33
#define ICM42688_APEX_DATA3                0x34
#define ICM42688_APEX_DATA4                0x35
#define ICM42688_APEX_DATA5                0x36
#define ICM42688_INT_STATUS2               0x37
#define ICM42688_INT_STATUS3               0x38
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_INTF_CONFIG1              0x4D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_TMST_CONFIG               0x54
#define ICM42688_APEX_CONFIG0              0x56
#define ICM42688_SMD_CONFIG                0x57
#define ICM42688_FIFO_CONFIG1              0x5F
#define ICM42688_FIFO_CONFIG2              0x60
#define ICM42688_FIFO_CONFIG3              0x61
#define ICM42688_FSYNC_CONFIG              0x62
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_CONFIG1               0x64
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_INT_SOURCE1               0x66
#define ICM42688_INT_SOURCE3               0x68
#define ICM42688_INT_SOURCE4               0x69
#define ICM42688_FIFO_LOST_PKT0            0x6C
#define ICM42688_FIFO_LOST_PKT1            0x6D
#define ICM42688_SELF_TEST_CONFIG          0x70
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76

/* INT_CONFIG 位定义 */
#define ICM42688_INT1_MODE_LATCH           0x04
#define ICM42688_INT1_DRIVE_OPEN_DRAIN     0x02
#define ICM42688_INT1_POLARITY_HIGH        0x01

/* INT_SOURCE0 位定义 */
#define ICM42688_UI_DRDY_INT1_EN           0x08

/* 加速度计量程选择 */
#define AFS_2G                             0x03
#define AFS_4G                             0x02
#define AFS_8G                             0x01
#define AFS_16G                            0x00

/* 陀螺仪量程选择 */
#define GFS_2000DPS                        0x00
#define GFS_1000DPS                        0x01
#define GFS_500DPS                         0x02
#define GFS_250DPS                         0x03
#define GFS_125DPS                         0x04
#define GFS_62_5DPS                        0x05
#define GFS_31_25DPS                       0x06
#define GFS_15_625DPS                      0x07

/* 加速度计输出数据率选择 */
#define AODR_8000Hz                        0x03
#define AODR_4000Hz                        0x04
#define AODR_2000Hz                        0x05
#define AODR_1000Hz                        0x06
#define AODR_200Hz                         0x07
#define AODR_100Hz                         0x08
#define AODR_50Hz                          0x09
#define AODR_25Hz                          0x0A
#define AODR_12_5Hz                        0x0B
#define AODR_6_25Hz                        0x0C
#define AODR_3_125Hz                       0x0D
#define AODR_1_5625Hz                      0x0E
#define AODR_500Hz                         0x0F

/* 陀螺仪输出数据率选择 */
#define GODR_8000Hz                        0x03
#define GODR_4000Hz                        0x04
#define GODR_2000Hz                        0x05
#define GODR_1000Hz                        0x06
#define GODR_200Hz                         0x07
#define GODR_100Hz                         0x08
#define GODR_50Hz                          0x09
#define GODR_25Hz                          0x0A
#define GODR_12_5Hz                        0x0B
#define GODR_500Hz                         0x0F

/**
 * @brief 原始三轴 IMU 数据
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm42688_raw_data_t;

/**
 * @brief 换算后的三轴 IMU 数据
 */
typedef struct {
    float x;
    float y;
    float z;
} icm42688_real_data_t;

/**
 * @brief 初始化 ICM42688P
 * @return 成功返回 1，失败返回 0
 */
uint8_t ICM_Init(void);

/**
 * @brief 读取 ICM42688P 设备 ID
 * @return 成功返回 1，失败返回 0
 */
uint8_t ICM_ReadID(void);

/**
 * @brief 更新加速度计灵敏度
 * @param scale 量程选择
 * @return 灵敏度，单位 g/LSB
 */
float bsp_Icm42688GetAres(uint8_t scale);

/**
 * @brief 更新陀螺仪灵敏度
 * @param scale 量程选择
 * @return 灵敏度，单位 dps/LSB
 */
float bsp_Icm42688GetGres(uint8_t scale);

/**
 * @brief 读取原始加速度计数据
 * @param data 输出缓冲区
 * @return 成功返回 0
 */
int8_t bsp_IcmGet_Accelerometer(icm42688_raw_data_t *data);

/**
 * @brief 读取原始陀螺仪数据
 * @param data 输出缓冲区
 * @return 成功返回 0
 */
int8_t bsp_IcmGet_Gyroscope(icm42688_raw_data_t *data);

/**
 * @brief 读取换算后的加速度计和陀螺仪数据
 * @param acc_data 输出加速度计缓冲区，单位 g
 * @param gyro_data 输出陀螺仪缓冲区，单位 dps
 * @return 成功返回 0
 */
int8_t bsp_IcmGetRawData(icm42688_real_data_t *acc_data, icm42688_real_data_t *gyro_data);

/**
 * @brief 将 UI 数据就绪中断路由到 INT1
 * @return 成功返回 0
 */
int8_t bsp_IcmConfigDataReadyInt(void);

/**
 * @brief 读取芯片温度
 * @param temp 输出温度，单位摄氏度
 * @return 成功返回 0
 */
int8_t bsp_IcmGet_Temperature(int16_t *temp);

#endif /* __BSP_ICM42688_H */
