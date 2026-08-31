/**
 * @file bsp_bmi088.c
 * @brief HAL SPI implementation for the BMI088 IMU.
 */

#include "bsp_bmi088.h"
#include "bsp_bmi088_reg.h"

#include <stddef.h>
#include <stdint.h>

#define BMI088_ACCEL_SEN       0.0008974358974f
#define BMI088_GYRO_SEN        0.0010652644360316953f
#define BMI088_TEMP_FACTOR     0.125f
#define BMI088_TEMP_OFFSET     23.0f
#define BMI088_INIT_DELAY_MS   80U
#define BMI088_COM_WAIT_US     150U

typedef struct
{
    uint8_t reg;
    uint8_t value;
    bsp_bmi088_err_t err;
} bmi088_reg_cfg_t;

static const bmi088_reg_cfg_t bmi088_acc_cfg[] =
{
    {BMI088_ACC_PWR_CTRL, (uint8_t)BMI088_ACC_ENABLE_ACC_ON, BSP_BMI088_ERR_ACC_PWR_CTRL},
    {BMI088_ACC_PWR_CONF, (uint8_t)BMI088_ACC_PWR_ACTIVE_MODE, BSP_BMI088_ERR_ACC_PWR_CONF},
    {BMI088_ACC_CONF, (uint8_t)(BMI088_ACC_NORMAL | BMI088_ACC_800_HZ |
                                BMI088_ACC_CONF_MUST_SET), BSP_BMI088_ERR_ACC_CONF},
    {BMI088_ACC_RANGE, (uint8_t)BMI088_ACC_RANGE_3G, BSP_BMI088_ERR_ACC_RANGE},
    {BMI088_INT1_IO_CTRL, (uint8_t)(BMI088_ACC_INT1_IO_ENABLE |
                                    BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW),
     BSP_BMI088_ERR_ACC_INT},
    {BMI088_INT_MAP_DATA, (uint8_t)BMI088_ACC_INT1_DRDY_INTERRUPT, BSP_BMI088_ERR_ACC_MAP}
};

static const bmi088_reg_cfg_t bmi088_gyro_cfg[] =
{
    {BMI088_GYRO_RANGE, (uint8_t)BMI088_GYRO_2000, BSP_BMI088_ERR_GYRO_RANGE},
    {BMI088_GYRO_BANDWIDTH, (uint8_t)(BMI088_GYRO_1000_116_HZ |
                                      BMI088_GYRO_BANDWIDTH_MUST_SET), BSP_BMI088_ERR_GYRO_BW},
    {BMI088_GYRO_LPM1, (uint8_t)BMI088_GYRO_NORMAL_MODE, BSP_BMI088_ERR_GYRO_POWER},
    {BMI088_GYRO_CTRL, (uint8_t)BMI088_DRDY_ON, BSP_BMI088_ERR_GYRO_CTRL},
    {BMI088_GYRO_INT3_INT4_IO_CONF, (uint8_t)(BMI088_GYRO_INT3_GPIO_PP |
                                              BMI088_GYRO_INT3_GPIO_LOW),
     BSP_BMI088_ERR_GYRO_INT},
    {BMI088_GYRO_INT3_INT4_IO_MAP, (uint8_t)BMI088_GYRO_DRDY_IO_INT3,
     BSP_BMI088_ERR_GYRO_MAP}
};

static uint8_t bmi088_valid(const bsp_bmi088_t *dev)
{
    return (dev != NULL) && (dev->spi != NULL) && (dev->spi->Instance != NULL) &&
           (dev->accel_cs_port != NULL) && (dev->gyro_cs_port != NULL) &&
           (dev->accel_cs_pin != 0U) && (dev->gyro_cs_pin != 0U);
}

static uint8_t bmi088_pin_valid(uint16_t pin)
{
    return (pin != 0U) && ((pin & (uint16_t)(pin - 1U)) == 0U);
}

static void bmi088_delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (us == 0U)
    {
        return;
    }

    /* DWT provides the short settling delay required between BMI088 SPI accesses. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    start = DWT->CYCCNT;
    ticks = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks)
    {
    }
}

static void bmi088_cs(const bsp_bmi088_t *dev, uint8_t accel, GPIO_PinState state)
{
    if (accel != 0U)
    {
        HAL_GPIO_WritePin(dev->accel_cs_port, dev->accel_cs_pin, state);
    }
    else
    {
        HAL_GPIO_WritePin(dev->gyro_cs_port, dev->gyro_cs_pin, state);
    }
}

static bsp_bmi088_err_t bmi088_xfer(const bsp_bmi088_t *dev, uint8_t tx, uint8_t *rx)
{
    uint8_t value = 0U;

    if (HAL_SPI_TransmitReceive(dev->spi, &tx, &value, 1U, 1000U) != HAL_OK)
    {
        return BSP_BMI088_ERR_SPI;
    }
    if (rx != NULL)
    {
        *rx = value;
    }
    return BSP_BMI088_OK;
}

static bsp_bmi088_err_t bmi088_write(const bsp_bmi088_t *dev, uint8_t accel,
                                     uint8_t reg, uint8_t value)
{
    bsp_bmi088_err_t err;

    bmi088_cs(dev, accel, GPIO_PIN_RESET);
    err = bmi088_xfer(dev, reg, NULL);
    if (err == BSP_BMI088_OK)
    {
        err = bmi088_xfer(dev, value, NULL);
    }
    bmi088_cs(dev, accel, GPIO_PIN_SET);
    bmi088_delay_us(BMI088_COM_WAIT_US);
    return err;
}

static bsp_bmi088_err_t bmi088_read(const bsp_bmi088_t *dev, uint8_t accel,
                                    uint8_t reg, uint8_t *buf, uint8_t len)
{
    bsp_bmi088_err_t err;
    uint8_t i;

    if ((buf == NULL) || (len == 0U))
    {
        return BSP_BMI088_ERR_ARG;
    }

    bmi088_cs(dev, accel, GPIO_PIN_RESET);
    err = bmi088_xfer(dev, (uint8_t)(reg | 0x80U), NULL);
    if ((err == BSP_BMI088_OK) && (accel != 0U))
    {
        err = bmi088_xfer(dev, 0x55U, NULL);
    }
    for (i = 0U; (i < len) && (err == BSP_BMI088_OK); ++i)
    {
        err = bmi088_xfer(dev, 0x55U, &buf[i]);
    }
    bmi088_cs(dev, accel, GPIO_PIN_SET);
    return err;
}

static bsp_bmi088_err_t bmi088_check_cfg(const bsp_bmi088_t *dev,
                                         uint8_t accel,
                                         const bmi088_reg_cfg_t *cfg,
                                         uint8_t count)
{
    uint8_t value;
    uint8_t i;
    bsp_bmi088_err_t err;

    for (i = 0U; i < count; ++i)
    {
        err = bmi088_write(dev, accel, cfg[i].reg, cfg[i].value);
        if (err != BSP_BMI088_OK)
        {
            return err;
        }
        err = bmi088_read(dev, accel, cfg[i].reg, &value, 1U);
        if (err != BSP_BMI088_OK)
        {
            return err;
        }
        if (value != cfg[i].value)
        {
            return cfg[i].err;
        }
    }
    return BSP_BMI088_OK;
}

static bsp_bmi088_err_t bmi088_init_accel(const bsp_bmi088_t *dev)
{
    uint8_t id;
    bsp_bmi088_err_t err;

    err = bmi088_read(dev, 1U, BMI088_ACC_CHIP_ID, &id, 1U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    bmi088_delay_us(BMI088_COM_WAIT_US);
    err = bmi088_read(dev, 1U, BMI088_ACC_CHIP_ID, &id, 1U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    err = bmi088_write(dev, 1U, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    HAL_Delay(BMI088_INIT_DELAY_MS);
    err = bmi088_read(dev, 1U, BMI088_ACC_CHIP_ID, &id, 1U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    bmi088_delay_us(BMI088_COM_WAIT_US);
    err = bmi088_read(dev, 1U, BMI088_ACC_CHIP_ID, &id, 1U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    if (id != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BSP_BMI088_ERR_ACC_ID;
    }
    return bmi088_check_cfg(dev, 1U, bmi088_acc_cfg,
                            (uint8_t)(sizeof(bmi088_acc_cfg) / sizeof(bmi088_acc_cfg[0])));
}

static bsp_bmi088_err_t bmi088_init_gyro(const bsp_bmi088_t *dev)
{
    uint8_t id;
    bsp_bmi088_err_t err;

    err = bmi088_read(dev, 0U, BMI088_GYRO_CHIP_ID, &id, 1U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    bmi088_delay_us(BMI088_COM_WAIT_US);
    err = bmi088_read(dev, 0U, BMI088_GYRO_CHIP_ID, &id, 1U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    err = bmi088_write(dev, 0U, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    HAL_Delay(BMI088_INIT_DELAY_MS);
    err = bmi088_read(dev, 0U, BMI088_GYRO_CHIP_ID, &id, 1U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    bmi088_delay_us(BMI088_COM_WAIT_US);
    err = bmi088_read(dev, 0U, BMI088_GYRO_CHIP_ID, &id, 1U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    if (id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BSP_BMI088_ERR_GYRO_ID;
    }
    return bmi088_check_cfg(dev, 0U, bmi088_gyro_cfg,
                            (uint8_t)(sizeof(bmi088_gyro_cfg) / sizeof(bmi088_gyro_cfg[0])));
}

bsp_bmi088_err_t BSP_BMI088_Init(bsp_bmi088_t *dev, SPI_HandleTypeDef *spi,
                                 GPIO_TypeDef *accel_cs_port, uint16_t accel_cs_pin,
                                 GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin)
{
    bsp_bmi088_err_t err;

    if ((dev == NULL) || (spi == NULL) || (spi->Instance == NULL) ||
        (accel_cs_port == NULL) || (gyro_cs_port == NULL) ||
        (bmi088_pin_valid(accel_cs_pin) == 0U) ||
        (bmi088_pin_valid(gyro_cs_pin) == 0U))
    {
        return BSP_BMI088_ERR_ARG;
    }

    dev->spi = spi;
    dev->accel_cs_port = accel_cs_port;
    dev->accel_cs_pin = accel_cs_pin;
    dev->gyro_cs_port = gyro_cs_port;
    dev->gyro_cs_pin = gyro_cs_pin;
    dev->initialized = 0U;
    bmi088_cs(dev, 1U, GPIO_PIN_SET);
    bmi088_cs(dev, 0U, GPIO_PIN_SET);

    err = bmi088_init_accel(dev);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    err = bmi088_init_gyro(dev);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    dev->initialized = 1U;
    return BSP_BMI088_OK;
}

bsp_bmi088_err_t BSP_BMI088_ReadId(const bsp_bmi088_t *dev,
                                   uint8_t *acc_id, uint8_t *gyro_id)
{
    bsp_bmi088_err_t err;

    if ((bmi088_valid(dev) == 0U) || ((acc_id == NULL) && (gyro_id == NULL)))
    {
        return BSP_BMI088_ERR_ARG;
    }
    if (acc_id != NULL)
    {
        err = bmi088_read(dev, 1U, BMI088_ACC_CHIP_ID, acc_id, 1U);
        if (err != BSP_BMI088_OK)
        {
            return err;
        }
    }
    if (gyro_id != NULL)
    {
        err = bmi088_read(dev, 0U, BMI088_GYRO_CHIP_ID, gyro_id, 1U);
        if (err != BSP_BMI088_OK)
        {
            return err;
        }
    }
    return BSP_BMI088_OK;
}

bsp_bmi088_err_t BSP_BMI088_Read(const bsp_bmi088_t *dev, bsp_bmi088_data_t *data)
{
    uint8_t buf[8];
    uint8_t temp_buf[2];
    int16_t raw;
    bsp_bmi088_err_t err;

    if ((bmi088_valid(dev) == 0U) || (dev->initialized == 0U) || (data == NULL))
    {
        return BSP_BMI088_ERR_ARG;
    }

    err = bmi088_read(dev, 1U, BMI088_ACCEL_XOUT_L, buf, 6U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
    data->accel[0] = (float)raw * BMI088_ACCEL_SEN;
    raw = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
    data->accel[1] = (float)raw * BMI088_ACCEL_SEN;
    raw = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));
    data->accel[2] = (float)raw * BMI088_ACCEL_SEN;

    err = bmi088_read(dev, 0U, BMI088_GYRO_CHIP_ID, buf, 8U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    if (buf[0] != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BSP_BMI088_ERR_GYRO_ID;
    }
    raw = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8U));
    data->gyro[0] = (float)raw * BMI088_GYRO_SEN;
    raw = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8U));
    data->gyro[1] = (float)raw * BMI088_GYRO_SEN;
    raw = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8U));
    data->gyro[2] = (float)raw * BMI088_GYRO_SEN;

    err = bmi088_read(dev, 1U, BMI088_TEMP_M, temp_buf, 2U);
    if (err != BSP_BMI088_OK)
    {
        return err;
    }
    raw = (int16_t)(((uint16_t)temp_buf[0] << 3U) | ((uint16_t)temp_buf[1] >> 5U));
    if (raw > 1023)
    {
        raw = (int16_t)(raw - 2048);
    }
    data->temperature = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    return BSP_BMI088_OK;
}
