/**
 * @file    imu.h
 * @brief   IMU attitude fusion interface.
 */

#ifndef __IMU_H
#define __IMU_H

#include <math.h>
#include <stdint.h>

/**
 * @brief 3-axis float vector.
 */
typedef struct {
    float x;
    float y;
    float z;
} xyz_f_t;

extern xyz_f_t north, west;
extern volatile float yaw[5];

/**
 * @brief Initialize ICM42688P and fusion state.
 */
void IMU_init(void);

/**
 * @brief Sample IMU data and update attitude.
 * @note  This function is triggered by the ICM42688P data-ready interrupt.
 */
void IMU_sample(void);

/**
 * @brief Handle the PA16 data-ready GPIO interrupt.
 */
void IMU_dataReadyIrqHandler(void);

/**
 * @brief Reset the IMU sample timestamp base.
 */
void IMU_resetTimestamp(void);

/**
 * @brief  Get the latest yaw, pitch and roll angles.
 * @param  angles Output array, index 0/1/2 maps to yaw/pitch/roll in degrees.
 * @return None.
 */
void IMU_getYawPitchRoll(float *angles);

#endif /* __IMU_H */
