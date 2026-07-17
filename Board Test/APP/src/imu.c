/**
 * @file    imu.c
 * @brief   基于 Mahony AHRS 的 IMU 姿态估计
 */

#include "imu.h"
#include "bsp_icm42688.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

/** @brief 由 ICM42688P 200 Hz 输出数据率换算得到的采样周期 */
#define IMU_SAMPLE_PERIOD_S       (0.005f)
/** @brief 四元数积分使用的半采样周期 */
#define IMU_HALF_SAMPLE_PERIOD_S  (IMU_SAMPLE_PERIOD_S * 0.5f)
/** @brief 陀螺仪零偏估计窗口大小 */
#define GYRO_CAL_SAMPLES          (100U)
/** @brief 静止方差阈值，单位 dps^2 */
#define GYRO_STABLE_VAR_TH        (0.01f)
/** @brief Mahony 比例系数 */
#define Kp                        (0.6f)
/** @brief Mahony 积分系数 */
#define Ki                        (0.001f)

/** @brief 最新的加速度计采样值，单位 g */
icm42688_real_data_t accval;
/** @brief 最新的陀螺仪采样值，单位 dps */
icm42688_real_data_t gyroval;
/** @brief 采样节拍计数器，1 tick 对应 1 次 IMU 数据就绪事件 */
volatile uint32_t nowtime = 0;

xyz_f_t north, west;
volatile float yaw[5] = {0.0f};

/** @brief Mahony 积分误差项 */
static volatile float exInt;
static volatile float eyInt;
static volatile float ezInt;
/** @brief 当前姿态四元数 */
static volatile float q0;
static volatile float q1;
static volatile float q2;
static volatile float q3;
/** @brief 上一次 AHRS 更新使用的节拍值 */
static volatile uint32_t lastUpdate;
static volatile uint32_t now;
/** @brief 缓存的欧拉角，顺序为 yaw、pitch、roll */
static volatile float ypr[3];
/** @brief 陀螺仪零偏完成稳定估计后的标志 */
static volatile uint8_t imu_ready;

/** @brief 用于零偏估计的滚动陀螺窗口 */
static double gyro_hist[3][GYRO_CAL_SAMPLES];
/** @brief 滚动陀螺总和 */
static double gyro_sum[3];
/** @brief 滚动陀螺平方和 */
static double gyro_sqr_sum[3];
/** @brief 陀螺仪标定零偏，单位 dps */
static float gyro_offset[3];
/** @brief 滚动窗口写索引 */
static uint16_t gyro_idx;
/** @brief 窗口填满标志 */
static uint8_t gyro_win_ready;

/**
 * @brief 快速反平方根
 * @param x 输入值
 * @return 1 / sqrt(x)
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
 * @brief 将数值限制到闭区间
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的数值
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
 * @brief 重置四元数与积分状态
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
 * @brief 在滚动静止窗口内更新陀螺仪方差
 * @param data 当前陀螺仪采样值，单位 dps
 * @param var 输出的 xyz 轴方差
 * @param avg 输出的 xyz 轴平均值
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
 * @brief 使用 Mahony 6 轴融合更新四元数
 * @param gx 陀螺仪 x 轴角速度，单位 rad/s
 * @param gy 陀螺仪 y 轴角速度，单位 rad/s
 * @param gz 陀螺仪 z 轴角速度，单位 rad/s
 * @param ax 加速度 x 轴，单位 g
 * @param ay 加速度 y 轴，单位 g
 * @param az 加速度 z 轴，单位 g
 * @param halfT 半个积分周期，单位秒
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
 * @brief 根据当前四元数更新缓存的欧拉角
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

void IMU_dataReadyIrqHandler(void)
{
    nowtime += 1U;
    IMU_sample();
}

void IMU_getYawPitchRoll(float *angles)
{
    __disable_irq();
    angles[0] = ypr[0];
    angles[1] = ypr[1];
    angles[2] = ypr[2];
    __enable_irq();
}
