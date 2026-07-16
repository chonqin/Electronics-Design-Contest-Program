/**
 * @file    bsp_icm42688.h
 * @brief   ICM42688 六轴惯性传感器驱动头文件
 * @note    适配TI MSPM0G3507平台,使用SPI通信
 */

#ifndef __BSP_ICM42688_H
#define __BSP_ICM42688_H

#include <stdint.h>
#include "board.h"

/* ICM42688设备ID */
#define ICM_DEVICE_ID                      0x47

/* Bank 0 寄存器地址 */
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

/* 加速度计量程配置 */
#define AFS_2G                             0x03
#define AFS_4G                             0x02
#define AFS_8G                             0x01
#define AFS_16G                            0x00

/* 陀螺仪量程配置 */
#define GFS_2000DPS                        0x00
#define GFS_1000DPS                        0x01
#define GFS_500DPS                         0x02
#define GFS_250DPS                         0x03
#define GFS_125DPS                         0x04
#define GFS_62_5DPS                        0x05
#define GFS_31_25DPS                       0x06
#define GFS_15_625DPS                      0x07

/* 加速度计输出速率配置 */
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

/* 陀螺仪输出速率配置 */
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
 * @brief 原始数据结构体(16位整型)
 */
typedef struct {
    int16_t x; /**< X轴原始数据 */
    int16_t y; /**< Y轴原始数据 */
    int16_t z; /**< Z轴原始数据 */
} icm42688_raw_data_t;

/**
 * @brief 实际物理量数据结构体(浮点型)
 */
typedef struct {
    float x; /**< X轴物理量 */
    float y; /**< Y轴物理量 */
    float z; /**< Z轴物理量 */
} icm42688_real_data_t;

/* 函数声明 */

/**
 * @brief  ICM42688初始化
 * @return 1=成功, 0=失败
 */
uint8_t ICM_Init(void);

/**
 * @brief  读取ICM42688设备ID
 * @return 1=成功, 0=失败
 */
uint8_t ICM_ReadID(void);

/**
 * @brief  设置加速度计灵敏度系数
 * @param  scale 量程配置(AFS_2G/AFS_4G/AFS_8G/AFS_16G)
 * @return 灵敏度系数(g/LSB)
 */
float bsp_Icm42688GetAres(uint8_t scale);

/**
 * @brief  设置陀螺仪灵敏度系数
 * @param  scale 量程配置(GFS_xxx)
 * @return 灵敏度系数(dps/LSB)
 */
float bsp_Icm42688GetGres(uint8_t scale);

/**
 * @brief  读取加速度计原始数据
 * @param  data 存储数据的结构体指针
 * @return 0=成功
 */
int8_t bsp_IcmGet_Accelerometer(icm42688_raw_data_t* data);

/**
 * @brief  读取陀螺仪原始数据
 * @param  data 存储数据的结构体指针
 * @return 0=成功
 */
int8_t bsp_IcmGet_Gyroscope(icm42688_raw_data_t* data);

/**
 * @brief  读取加速度计和陀螺仪的实际物理量
 * @param  acc_data  加速度数据指针(单位:g)
 * @param  gyro_data 陀螺仪数据指针(单位:dps)
 * @return 0=成功
 */
int8_t bsp_IcmGetRawData(icm42688_real_data_t* acc_data, icm42688_real_data_t* gyro_data);

/**
 * @brief  读取温度
 * @param  temp 温度指针(单位:℃)
 * @return 0=成功
 */
int8_t bsp_IcmGet_Temperature(int16_t* temp);

#endif /* __BSP_ICM42688_H */
