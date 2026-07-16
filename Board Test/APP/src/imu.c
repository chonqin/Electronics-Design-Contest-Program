/**
 * @file    imu.c
 * @brief   IMU attitude estimation based on Mahony AHRS.
 */

#include "imu.h"
#include "bsp_icm42688.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

/** @brief IMU sample period configured in SysConfig. */
#define IMU_SAMPLE_PERIOD_S       (0.010f)
/** @brief Half sample period used by quaternion integration. */
#define IMU_HALF_SAMPLE_PERIOD_S  (IMU_SAMPLE_PERIOD_S * 0.5f)
/** @brief Gyro offset window size. */
#define GYRO_CAL_SAMPLES          (100U)
/** @brief Stationary variance threshold in dps^2. */
#define GYRO_STABLE_VAR_TH        (0.01f)
/** @brief Mahony proportional gain. */
#define Kp                        (0.6f)
/** @brief Mahony integral gain. */
#define Ki                        (0.001f)

/** @brief Latest accelerometer sample in g. */
icm42688_real_data_t accval;
/** @brief Latest gyroscope sample in dps. */
icm42688_real_data_t gyroval;
/** @brief Sample tick counter, 1 tick = 10 ms. */
volatile uint32_t nowtime = 0;

xyz_f_t north, west;
volatile float yaw[5] = {0.0f};

/** @brief Mahony integral error terms. */
static volatile float exInt;
static volatile float eyInt;
static volatile float ezInt;
/** @brief Current attitude quaternion. */
static volatile float q0;
static volatile float q1;
static volatile float q2;
static volatile float q3;
/** @brief Sample tick used by the last AHRS update. */
static volatile uint32_t lastUpdate;
static volatile uint32_t now;
/** @brief Cached Euler angles in yaw, pitch, roll order. */
static volatile float ypr[3];
/** @brief Set after gyro offset has a stable stationary estimate. */
static volatile uint8_t imu_ready;

/** @brief Rolling gyro window for offset estimation. */
static double gyro_hist[3][GYRO_CAL_SAMPLES];
/** @brief Rolling gyro sum. */
static double gyro_sum[3];
/** @brief Rolling gyro squared sum. */
static double gyro_sqr_sum[3];
/** @brief Gyro calibration offset in dps. */
static float gyro_offset[3];
/** @brief Rolling window write index. */
static uint16_t gyro_idx;
/** @brief Window filled flag. */
static uint8_t gyro_win_ready;

/**
 * @brief  Fast inverse square root.
 * @param  x Input value.
 * @return 1 / sqrt(x).
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;

    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

/**
 * @brief  Clamp a value to a closed interval.
 * @param  val Input value.
 * @param  min Lower bound.
 * @param  max Upper bound.
 * @return Clamped value.
 */
static float imu_clampf(float val, float min, float max)
{
    if (val < min) {
        return min;
    }
    if (val > max) {
        return max;
    }
    return val;
}

/**
 * @brief Reset quaternion and integration state.
 */
static void IMU_resetFusion(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
    ypr[0] = 0.0f;
    ypr[1] = 0.0f;
    ypr[2] = 0.0f;
}

/**
 * @brief  Update gyro variance over a rolling stationary window.
 * @param  data Current gyro sample in dps.
 * @param  var Output variance for xyz axes.
 * @param  avg Output average for xyz axes.
 */
static void calGyroVariance(const float data[3], float var[3], float avg[3])
{
    uint8_t i;

    if (gyro_win_ready == 0U) {
        for (i = 0U; i < 3U; i++) {
            gyro_hist[i][gyro_idx] = data[i];
            gyro_sum[i] += data[i];
            gyro_sqr_sum[i] += data[i] * data[i];
            var[i] = 100.0f;
            avg[i] = 0.0f;
        }
    } else {
        for (i = 0U; i < 3U; i++) {
            gyro_sum[i] -= gyro_hist[i][gyro_idx];
            gyro_sqr_sum[i] -= gyro_hist[i][gyro_idx] * gyro_hist[i][gyro_idx];
            gyro_hist[i][gyro_idx] = data[i];
            gyro_sum[i] += gyro_hist[i][gyro_idx];
            gyro_sqr_sum[i] += gyro_hist[i][gyro_idx] * gyro_hist[i][gyro_idx];
        }
    }

    gyro_idx++;
    if (gyro_idx >= GYRO_CAL_SAMPLES) {
        gyro_idx = 0U;
        gyro_win_ready = 1U;
    }

    if (gyro_win_ready == 0U) {
        return;
    }

    for (i = 0U; i < 3U; i++) {
        avg[i] = (float)(gyro_sum[i] / (double)GYRO_CAL_SAMPLES);
        var[i] = (float)((gyro_sqr_sum[i] -
                 gyro_sum[i] * gyro_sum[i] / (double)GYRO_CAL_SAMPLES) /
                 (double)GYRO_CAL_SAMPLES);
    }
}

/**
 * @brief  Update quaternion with Mahony 6-axis fusion.
 * @param  gx Gyro x in rad/s.
 * @param  gy Gyro y in rad/s.
 * @param  gz Gyro z in rad/s.
 * @param  ax Acc x in g.
 * @param  ay Acc y in g.
 * @param  az Acc z in g.
 * @param  halfT Half of the integration period in seconds.
 */
static void IMU_AHRSupdate(float gx, float gy, float gz,
                           float ax, float ay, float az, float halfT)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float tempq0, tempq1, tempq2, tempq3;
    float acc_mag2;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    acc_mag2 = ax * ax + ay * ay + az * az;
    if (acc_mag2 > 0.25f && acc_mag2 < 4.0f) {
        norm = invSqrt(acc_mag2);
        ax *= norm;
        ay *= norm;
        az *= norm;

        vx = 2.0f * (q1q3 - q0q2);
        vy = 2.0f * (q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        north.x = 1.0f - 2.0f * (q3q3 + q2q2);
        north.y = 2.0f * (-q0q3 + q1q2);
        north.z = 2.0f * (q0q2 - q1q3);
        west.x = 2.0f * (q0q3 + q1q2);
        west.y = 1.0f - 2.0f * (q3q3 + q1q1);
        west.z = 2.0f * (-q0q1 + q2q3);

        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        exInt += ex * Ki * halfT;
        eyInt += ey * Ki * halfT;
        ezInt += ez * Ki * halfT;

        gx += Kp * ex + exInt;
        gy += Kp * ey + eyInt;
        gz += Kp * ez + ezInt;
    }

    tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    tempq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    tempq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    tempq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = invSqrt(tempq0 * tempq0 + tempq1 * tempq1 +
                   tempq2 * tempq2 + tempq3 * tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

/**
 * @brief Update cached Euler angles from the current quaternion.
 */
static void IMU_updateYpr(void)
{
    float pitch_sin;

    pitch_sin = imu_clampf(-2.0f * q1 * q3 + 2.0f * q0 * q2, -1.0f, 1.0f);

    ypr[0] = -atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3,
                     -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / M_PI;
    ypr[1] = -asinf(pitch_sin) * 180.0f / M_PI;
    ypr[2] = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
                    -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 180.0f / M_PI;

    yaw[0] = ypr[0];
    yaw[1] = ypr[1];
    yaw[2] = ypr[2];
}

void IMU_resetTimestamp(void)
{
    __disable_irq();
    lastUpdate = nowtime;
    now = nowtime;
    __enable_irq();
}

void IMU_init(void)
{
    if (ICM_Init() != 1U) {
        printf("IMU Init failed!\r\n");
        return;
    }

    IMU_resetFusion();

    memset(gyro_hist, 0, sizeof(gyro_hist));
    memset(gyro_sum, 0, sizeof(gyro_sum));
    memset(gyro_sqr_sum, 0, sizeof(gyro_sqr_sum));
    memset(gyro_offset, 0, sizeof(gyro_offset));
    gyro_idx = 0U;
    gyro_win_ready = 0U;
    imu_ready = 0U;

    IMU_resetTimestamp();

    printf("IMU Init success!\r\n");
}

void IMU_sample(void)
{
    float gyro_var[3];
    float gyro_avg[3];
    float gyro_now[3];
    float halfT;
    uint32_t dt_tick;

    bsp_IcmGetRawData(&accval, &gyroval);

    gyro_now[0] = gyroval.x;
    gyro_now[1] = gyroval.y;
    gyro_now[2] = gyroval.z;
    calGyroVariance(gyro_now, gyro_var, gyro_avg);

    if ((gyro_win_ready != 0U) &&
        (gyro_var[0] < GYRO_STABLE_VAR_TH) &&
        (gyro_var[1] < GYRO_STABLE_VAR_TH) &&
        (gyro_var[2] < GYRO_STABLE_VAR_TH)) {
        gyro_offset[0] = gyro_avg[0];
        gyro_offset[1] = gyro_avg[1];
        gyro_offset[2] = gyro_avg[2];
        exInt = 0.0f;
        eyInt = 0.0f;
        ezInt = 0.0f;

        if (imu_ready == 0U) {
            imu_ready = 1U;
            lastUpdate = nowtime;
            IMU_resetFusion();
            return;
        }
    }

    if (imu_ready == 0U) {
        return;
    }

    now = nowtime;
    dt_tick = now - lastUpdate;
    lastUpdate = now;
    if (dt_tick == 0U) {
        return;
    }

    halfT = (float)dt_tick * IMU_HALF_SAMPLE_PERIOD_S;

    IMU_AHRSupdate((gyroval.x - gyro_offset[0]) * M_PI / 180.0f,
                   (gyroval.y - gyro_offset[1]) * M_PI / 180.0f,
                   (gyroval.z - gyro_offset[2]) * M_PI / 180.0f,
                   accval.x, accval.y, accval.z, halfT);
    IMU_updateYpr();
}

void IMU_getYawPitchRoll(float *angles)
{
    __disable_irq();
    angles[0] = ypr[0];
    angles[1] = ypr[1];
    angles[2] = ypr[2];
    __enable_irq();
}
