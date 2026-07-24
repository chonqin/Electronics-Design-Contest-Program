/**
 * @file    bsp_icm42688.c
 * @brief   ICM42688P SPI 驱动实现
 */

#include "bsp_icm42688.h"
#include <stdio.h>

/** @brief IMU 片选拉低 */
#define ICM_CS_LOW()   DL_GPIO_clearPins(GPIO_IMU_CS_PORT, GPIO_IMU_CS_PB25_PIN)
/** @brief IMU 片选拉高 */
#define ICM_CS_HIGH()  DL_GPIO_setPins(GPIO_IMU_CS_PORT, GPIO_IMU_CS_PB25_PIN)
/** @brief 毫秒级忙等待延时 */
#define ICM_DELAY_MS(ms)  delay_cycles((ms) * (CPUCLK_FREQ / 1000))

/** @brief 当前加速度计灵敏度，单位 g/LSB */
static float Acc_Sensitivity = 4.0f / 32768.0f;
/** @brief 当前陀螺仪灵敏度，单位 dps/LSB */
static float Gyro_Sensitivity = 1000.0f / 32768.0f;

/**
 * @brief 通过 SPI 发送和接收 1 字节
 * @param tx_data 待发送字节
 * @return 接收到的字节
 */
static uint8_t SPI_ReadWriteByte(uint8_t tx_data)
{
    uint8_t rx_data;

    DL_SPI_transmitDataBlocking8(SPI_IMU_INST, tx_data);
    rx_data = DL_SPI_receiveDataBlocking8(SPI_IMU_INST);

    return rx_data;
}

/**
 * @brief 读取一个寄存器
 * @param addr 寄存器地址
 * @return 寄存器值
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
 * @brief 读取一段寄存器块
 * @param addr 起始地址
 * @param buffer 目标缓冲区
 * @param len 读取长度，单位字节
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
 * @brief 写入一个寄存器
 * @param addr 寄存器地址
 * @param data 寄存器值
 * @return 始终返回 0
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
 * @brief 配置 INT1 的数据就绪中断输出
 * @return 始终返回 0
 */
int8_t bsp_IcmConfigDataReadyInt(void)
{
    /* INT1：推挽、有效高、脉冲模式 */
    ICM_Write_A_Byte(ICM42688_INT_CONFIG, ICM42688_INT1_POLARITY_HIGH);
    /* 使用默认异步复位行为，仅路由 UI 数据就绪中断 */
    ICM_Write_A_Byte(ICM42688_INT_CONFIG1, 0x00U);
    ICM_Write_A_Byte(ICM42688_INT_SOURCE0, ICM42688_UI_DRDY_INT1_EN);
    /* 配置后读一次，清除可能残留的状态 */
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

    // 先软复位并等待芯片重新上电完成。
    ICM_Write_A_Byte(ICM42688_REG_BANK_SEL, 0x00U);
    ICM_Write_A_Byte(ICM42688_DEVICE_CONFIG, 0x01U);
    ICM_DELAY_MS(100);

    temp = ICM_Read_A_Byte(ICM42688_WHO_AM_I);
    if (temp != ICM_DEVICE_ID) {
        printf("ICM42688 Init failed! ID=0x%02X\r\n", temp);
        return 0U;
    }

    ICM_DELAY_MS(10);

    // 加速度计和陀螺仪统一配置为 200Hz，方便姿态解算固定节拍。
    ICM_Write_A_Byte(ICM42688_REG_BANK_SEL, 0x00U);
    bsp_Icm42688GetAres(AFS_4G);
    temp = (uint8_t)((AFS_4G << 5) | AODR_200Hz);
    ICM_Write_A_Byte(ICM42688_ACCEL_CONFIG0, temp);

    ICM_Write_A_Byte(ICM42688_REG_BANK_SEL, 0x00U);
    bsp_Icm42688GetGres(GFS_1000DPS);
    temp = (uint8_t)((GFS_1000DPS << 5) | GODR_200Hz);
    ICM_Write_A_Byte(ICM42688_GYRO_CONFIG0, temp);

    ICM_Write_A_Byte(ICM42688_GYRO_ACCEL_CONFIG0, 0x44U);

    /* 使能温度、陀螺仪和加速度计，进入低噪声模式 */
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

    // 原始整数值在这里统一换算成物理单位，供上层直接使用。
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
