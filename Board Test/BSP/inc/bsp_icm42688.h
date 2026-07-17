/**
 * @file    bsp_icm42688.h
 * @brief   ICM42688P SPI driver interface.
 */

#ifndef __BSP_ICM42688_H
#define __BSP_ICM42688_H

#include <stdint.h>
#include "board.h"

/* Device ID */
#define ICM_DEVICE_ID                      0x47

/* Bank 0 registers */
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

/* INT_CONFIG bits */
#define ICM42688_INT1_MODE_LATCH           0x04
#define ICM42688_INT1_DRIVE_OPEN_DRAIN     0x02
#define ICM42688_INT1_POLARITY_HIGH        0x01

/* INT_SOURCE0 bits */
#define ICM42688_UI_DRDY_INT1_EN           0x08

/* Accelerometer full-scale selections */
#define AFS_2G                             0x03
#define AFS_4G                             0x02
#define AFS_8G                             0x01
#define AFS_16G                            0x00

/* Gyroscope full-scale selections */
#define GFS_2000DPS                        0x00
#define GFS_1000DPS                        0x01
#define GFS_500DPS                         0x02
#define GFS_250DPS                         0x03
#define GFS_125DPS                         0x04
#define GFS_62_5DPS                        0x05
#define GFS_31_25DPS                       0x06
#define GFS_15_625DPS                      0x07

/* Accelerometer output data rate selections */
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

/* Gyroscope output data rate selections */
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
 * @brief Raw 3-axis IMU data.
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm42688_raw_data_t;

/**
 * @brief Scaled 3-axis IMU data.
 */
typedef struct {
    float x;
    float y;
    float z;
} icm42688_real_data_t;

/**
 * @brief  Initialize the ICM42688P.
 * @return 1 on success, 0 on failure.
 */
uint8_t ICM_Init(void);

/**
 * @brief  Read the ICM42688P device ID.
 * @return 1 on success, 0 on failure.
 */
uint8_t ICM_ReadID(void);

/**
 * @brief  Update accelerometer sensitivity.
 * @param  scale Full-scale selection.
 * @return Sensitivity in g/LSB.
 */
float bsp_Icm42688GetAres(uint8_t scale);

/**
 * @brief  Update gyroscope sensitivity.
 * @param  scale Full-scale selection.
 * @return Sensitivity in dps/LSB.
 */
float bsp_Icm42688GetGres(uint8_t scale);

/**
 * @brief  Read raw accelerometer data.
 * @param  data Output buffer.
 * @return 0 on success.
 */
int8_t bsp_IcmGet_Accelerometer(icm42688_raw_data_t *data);

/**
 * @brief  Read raw gyroscope data.
 * @param  data Output buffer.
 * @return 0 on success.
 */
int8_t bsp_IcmGet_Gyroscope(icm42688_raw_data_t *data);

/**
 * @brief  Read scaled accelerometer and gyroscope data.
 * @param  acc_data Output accelerometer buffer in g.
 * @param  gyro_data Output gyroscope buffer in dps.
 * @return 0 on success.
 */
int8_t bsp_IcmGetRawData(icm42688_real_data_t *acc_data, icm42688_real_data_t *gyro_data);

/**
 * @brief  Route the UI data-ready interrupt to INT1.
 * @return 0 on success.
 */
int8_t bsp_IcmConfigDataReadyInt(void);

/**
 * @brief  Read the die temperature.
 * @param  temp Output temperature in Celsius.
 * @return 0 on success.
 */
int8_t bsp_IcmGet_Temperature(int16_t *temp);

#endif /* __BSP_ICM42688_H */
