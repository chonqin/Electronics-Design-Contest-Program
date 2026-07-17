/**
 * @file    bsp_icm42688.c
 * @brief   ICM42688P SPI driver implementation.
 */

#include "bsp_icm42688.h"
#include <stdio.h>

/** @brief IMU chip-select low level. */
#define ICM_CS_LOW()   DL_GPIO_clearPins(GPIO_IMU_CS_PORT, GPIO_IMU_CS_PB25_PIN)
/** @brief IMU chip-select high level. */
#define ICM_CS_HIGH()  DL_GPIO_setPins(GPIO_IMU_CS_PORT, GPIO_IMU_CS_PB25_PIN)
/** @brief Busy-wait delay in milliseconds. */
#define ICM_DELAY_MS(ms)  delay_cycles((ms) * (CPUCLK_FREQ / 1000))

/** @brief Current accelerometer sensitivity in g/LSB. */
static float Acc_Sensitivity = 4.0f / 32768.0f;
/** @brief Current gyroscope sensitivity in dps/LSB. */
static float Gyro_Sensitivity = 1000.0f / 32768.0f;

/**
 * @brief  Transfer one byte over SPI.
 * @param  tx_data Byte to transmit.
 * @return Received byte.
 */
static uint8_t SPI_ReadWriteByte(uint8_t tx_data)
{
    uint8_t rx_data;

    DL_SPI_transmitDataBlocking8(SPI_IMU_INST, tx_data);
    rx_data = DL_SPI_receiveDataBlocking8(SPI_IMU_INST);

    return rx_data;
}

/**
 * @brief  Read one register.
 * @param  addr Register address.
 * @return Register value.
 */
static uint8_t ICM_Read_A_Byte(uint8_t addr)
{
    uint8_t temp;

    ICM_CS_LOW();
    SPI_ReadWriteByte(addr | 0x80U);
    temp = SPI_ReadWriteByte(0x00U);
    ICM_CS_HIGH();

    return temp;
}

/**
 * @brief  Read a register block.
 * @param  addr Start address.
 * @param  buffer Destination buffer.
 * @param  len Read length in bytes.
 */
static void ICM_Read_Bytes(uint8_t addr, uint8_t *buffer, uint8_t len)
{
    ICM_CS_LOW();
    SPI_ReadWriteByte(addr | 0x80U);

    while (len--) {
        *buffer++ = SPI_ReadWriteByte(0x00U);
    }

    ICM_CS_HIGH();
}

/**
 * @brief  Write one register.
 * @param  addr Register address.
 * @param  data Register value.
 * @return Always returns 0.
 */
static uint8_t ICM_Write_A_Byte(uint8_t addr, uint8_t data)
{
    ICM_CS_LOW();
    SPI_ReadWriteByte(addr);
    SPI_ReadWriteByte(data);
    ICM_CS_HIGH();

    return 0;
}

/**
 * @brief Configure data-ready interrupt output on INT1.
 * @return Always returns 0.
 */
int8_t bsp_IcmConfigDataReadyInt(void)
{
    /* INT1: push-pull, active high, pulse mode. */
    ICM_Write_A_Byte(ICM42688_INT_CONFIG, ICM42688_INT1_POLARITY_HIGH);
    /* Use the default async reset behavior and route UI data-ready only. */
    ICM_Write_A_Byte(ICM42688_INT_CONFIG1, 0x00U);
    ICM_Write_A_Byte(ICM42688_INT_SOURCE0, ICM42688_UI_DRDY_INT1_EN);
    /* Read once to clear any stale status after configuration. */
    (void) ICM_Read_A_Byte(ICM42688_INT_STATUS);

    return 0;
}

uint8_t ICM_ReadID(void)
{
    return (ICM_Read_A_Byte(ICM42688_WHO_AM_I) == ICM_DEVICE_ID) ? 1U : 0U;
}

uint8_t ICM_Init(void)
{
    uint8_t temp;

    ICM_Write_A_Byte(ICM42688_REG_BANK_SEL, 0x00U);
    ICM_Write_A_Byte(ICM42688_DEVICE_CONFIG, 0x01U);
    ICM_DELAY_MS(100);

    temp = ICM_Read_A_Byte(ICM42688_WHO_AM_I);
    if (temp != ICM_DEVICE_ID) {
        printf("ICM42688 Init failed! ID=0x%02X\r\n", temp);
        return 0U;
    }

    ICM_DELAY_MS(10);

    ICM_Write_A_Byte(ICM42688_REG_BANK_SEL, 0x00U);
    bsp_Icm42688GetAres(AFS_4G);
    temp = (uint8_t)((AFS_4G << 5) | AODR_200Hz);
    ICM_Write_A_Byte(ICM42688_ACCEL_CONFIG0, temp);

    ICM_Write_A_Byte(ICM42688_REG_BANK_SEL, 0x00U);
    bsp_Icm42688GetGres(GFS_1000DPS);
    temp = (uint8_t)((GFS_1000DPS << 5) | GODR_200Hz);
    ICM_Write_A_Byte(ICM42688_GYRO_CONFIG0, temp);

    ICM_Write_A_Byte(ICM42688_GYRO_ACCEL_CONFIG0, 0x44U);

    /* Enable temperature, gyro and accel in low-noise mode. */
    temp = ICM_Read_A_Byte(ICM42688_PWR_MGMT0);
    temp &= (uint8_t)~(1U << 5);
    temp &= (uint8_t)~0x0FU;
    temp |= (3U << 2);
    temp |= 3U;
    ICM_Write_A_Byte(ICM42688_PWR_MGMT0, temp);
    ICM_DELAY_MS(10);

    bsp_IcmConfigDataReadyInt();

    printf("ICM42688 Init success!\r\n");
    return 1U;
}

int8_t bsp_IcmGet_Temperature(int16_t *temp)
{
    uint8_t buffer[2];

    ICM_Read_Bytes(ICM42688_TEMP_DATA1, buffer, 2);
    *temp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48f + 25.0f);

    return 0;
}

int8_t bsp_IcmGet_Accelerometer(icm42688_raw_data_t *data)
{
    uint8_t buffer[6];

    ICM_Read_Bytes(ICM42688_ACCEL_DATA_X1, buffer, 6);

    data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return 0;
}

int8_t bsp_IcmGet_Gyroscope(icm42688_raw_data_t *data)
{
    uint8_t buffer[6];

    ICM_Read_Bytes(ICM42688_GYRO_DATA_X1, buffer, 6);

    data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return 0;
}

int8_t bsp_IcmGetRawData(icm42688_real_data_t *acc_data, icm42688_real_data_t *gyro_data)
{
    uint8_t buffer[12];
    icm42688_raw_data_t acc_raw;
    icm42688_raw_data_t gyro_raw;

    ICM_Read_Bytes(ICM42688_ACCEL_DATA_X1, buffer, 12);

    acc_raw.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    acc_raw.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    acc_raw.z = (int16_t)((buffer[4] << 8) | buffer[5]);

    gyro_raw.x = (int16_t)((buffer[6] << 8) | buffer[7]);
    gyro_raw.y = (int16_t)((buffer[8] << 8) | buffer[9]);
    gyro_raw.z = (int16_t)((buffer[10] << 8) | buffer[11]);

    acc_data->x = acc_raw.x * Acc_Sensitivity;
    acc_data->y = acc_raw.y * Acc_Sensitivity;
    acc_data->z = acc_raw.z * Acc_Sensitivity;

    gyro_data->x = gyro_raw.x * Gyro_Sensitivity;
    gyro_data->y = gyro_raw.y * Gyro_Sensitivity;
    gyro_data->z = gyro_raw.z * Gyro_Sensitivity;

    return 0;
}

float bsp_Icm42688GetAres(uint8_t scale)
{
    switch (scale) {
        case AFS_2G:
            Acc_Sensitivity = 2.0f / 32768.0f;
            break;
        case AFS_4G:
            Acc_Sensitivity = 4.0f / 32768.0f;
            break;
        case AFS_8G:
            Acc_Sensitivity = 8.0f / 32768.0f;
            break;
        case AFS_16G:
            Acc_Sensitivity = 16.0f / 32768.0f;
            break;
        default:
            break;
    }

    return Acc_Sensitivity;
}

float bsp_Icm42688GetGres(uint8_t scale)
{
    switch (scale) {
        case GFS_15_625DPS:
            Gyro_Sensitivity = 15.625f / 32768.0f;
            break;
        case GFS_31_25DPS:
            Gyro_Sensitivity = 31.25f / 32768.0f;
            break;
        case GFS_62_5DPS:
            Gyro_Sensitivity = 62.5f / 32768.0f;
            break;
        case GFS_125DPS:
            Gyro_Sensitivity = 125.0f / 32768.0f;
            break;
        case GFS_250DPS:
            Gyro_Sensitivity = 250.0f / 32768.0f;
            break;
        case GFS_500DPS:
            Gyro_Sensitivity = 500.0f / 32768.0f;
            break;
        case GFS_1000DPS:
            Gyro_Sensitivity = 1000.0f / 32768.0f;
            break;
        case GFS_2000DPS:
            Gyro_Sensitivity = 2000.0f / 32768.0f;
            break;
        default:
            break;
    }

    return Gyro_Sensitivity;
}
