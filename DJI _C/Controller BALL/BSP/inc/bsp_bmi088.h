/**
 * @file bsp_bmi088.h
 * @brief HAL SPI driver API for the BMI088 IMU.
 */

#ifndef BSP_BMI088_H
#define BSP_BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/** @brief BMI088 driver result codes. */
typedef enum
{
    BSP_BMI088_OK = 0x00U,
    BSP_BMI088_ERR_ACC_PWR_CTRL = 0x01U,
    BSP_BMI088_ERR_ACC_PWR_CONF = 0x02U,
    BSP_BMI088_ERR_ACC_CONF = 0x03U,
    BSP_BMI088_ERR_ACC_RANGE = 0x05U,
    BSP_BMI088_ERR_ACC_INT = 0x06U,
    BSP_BMI088_ERR_ACC_MAP = 0x07U,
    BSP_BMI088_ERR_GYRO_RANGE = 0x08U,
    BSP_BMI088_ERR_GYRO_BW = 0x09U,
    BSP_BMI088_ERR_GYRO_POWER = 0x0AU,
    BSP_BMI088_ERR_GYRO_CTRL = 0x0BU,
    BSP_BMI088_ERR_GYRO_INT = 0x0CU,
    BSP_BMI088_ERR_GYRO_MAP = 0x0DU,
    BSP_BMI088_ERR_ARG = 0x20U,
    BSP_BMI088_ERR_SPI = 0x21U,
    BSP_BMI088_ERR_ACC_ID = 0x80U,
    BSP_BMI088_ERR_GYRO_ID = 0x40U
} bsp_bmi088_err_t;

/** @brief Scaled BMI088 values. Acceleration is in m/s^2 and rate is in rad/s. */
typedef struct
{
    float accel[3];
    float gyro[3];
    float temperature;
} bsp_bmi088_data_t;

/** @brief BMI088 device context. GPIOs must already be configured by CubeMX. */
typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *accel_cs_port;
    uint16_t accel_cs_pin;
    GPIO_TypeDef *gyro_cs_port;
    uint16_t gyro_cs_pin;
    uint8_t initialized;
} bsp_bmi088_t;

/**
 * @brief Initialize the BMI088 over an already configured SPI peripheral.
 * @param dev Device context to initialize.
 * @param spi HAL SPI handle configured for BMI088 SPI mode.
 * @param accel_cs_port Accelerometer chip-select GPIO port.
 * @param accel_cs_pin Accelerometer chip-select GPIO pin.
 * @param gyro_cs_port Gyroscope chip-select GPIO port.
 * @param gyro_cs_pin Gyroscope chip-select GPIO pin.
 * @return BSP_BMI088_OK or a specific initialization error.
 */
bsp_bmi088_err_t BSP_BMI088_Init(bsp_bmi088_t *dev, SPI_HandleTypeDef *spi,
                                 GPIO_TypeDef *accel_cs_port, uint16_t accel_cs_pin,
                                 GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin);

/**
 * @brief Read one complete scaled BMI088 sample.
 * @param dev Initialized device context.
 * @param data Destination for acceleration, angular rate and temperature.
 * @return BSP_BMI088_OK or a communication/argument error.
 */
bsp_bmi088_err_t BSP_BMI088_Read(const bsp_bmi088_t *dev, bsp_bmi088_data_t *data);

/**
 * @brief Read the accelerometer and gyroscope chip IDs.
 * @param dev Initialized device context.
 * @param acc_id Optional accelerometer ID output.
 * @param gyro_id Optional gyroscope ID output.
 * @return BSP_BMI088_OK or a communication/argument error.
 */
bsp_bmi088_err_t BSP_BMI088_ReadId(const bsp_bmi088_t *dev,
                                   uint8_t *acc_id, uint8_t *gyro_id);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BMI088_H */
