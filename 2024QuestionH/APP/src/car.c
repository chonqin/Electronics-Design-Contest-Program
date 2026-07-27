/**
 * @file car.c
 * @brief 直通 PWM duty 底盘控制实现
 */
#include "car.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_track.h"
#include "imu.h"
#include "pid.h"
#include "ti_msp_dl_config.h"
#include <math.h>

#define LEFT_MOTOR              MOTOR_B
#define RIGHT_MOTOR             MOTOR_A
#define LEFT_ENCODER            ENCODER_E2
#define RIGHT_ENCODER           ENCODER_E1
#define CAR_TRACK_KP            7.0f
#define CAR_TRACK_KI            0.04f
#define CAR_TRACK_KD            2.0f
#define CAR_TRACK_TURN_MAX      1000
#define CAR_TRACK_LOST_MAX      10U
#define CAR_YAW_KP              16.0f
#define CAR_YAW_KI              0.18f
#define CAR_YAW_KD              1.9f
#define CAR_YAW_TURN_MAX        1000
#define CAR_TURN_DONE_DEG       2.0f
#define CAR_DUTY_MIN            300

static const int track_weight[TRACK_NUM] = {
    -100, -70, -40, -10, 10, 40, 70, 100
};

typedef struct {
    Car_Status st;
    int base;
    int last_pos;
    float yaw_tar;
    uint8_t lost;
    uint8_t track_seen;
    uint8_t track_find;
    uint8_t imu_on;
    PID pid_track;
    PID pid_yaw;
} Car_State;

static Car_State car;

/**
 * @brief 将整数限制到闭区间
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的整数
 */
static int car_limit(int val, int min, int max)
{
    if (val > max) {
        return max;
    }
    if (val < min) {
        return min;
    }
    return val;
}

/**
 * @brief 补偿电机非零控制量的机械死区
 * @param duty 限幅后的 PWM duty
 * @return 保留方向且绝对值不低于最小有效 duty 的命令，零命令保持不变
 */
static int car_deadzone(int duty)
{
    if ((duty > 0) && (duty < CAR_DUTY_MIN)) {
        return CAR_DUTY_MIN;
    }
    if ((duty < 0) && (duty > -CAR_DUTY_MIN)) {
        return -CAR_DUTY_MIN;
    }
    return duty;
}

/**
 * @brief 将角度归一化到 [-180, 180]
 * @param deg 输入角度，单位为度
 * @return 归一化后的角度
 */
static float car_wrap_deg(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

/** @brief 复位当前动作的运行状态，保留传感器反馈 */
static void car_reset_action(void)
{
    car.st.done = 0U;
    car.base = 0;
    car.last_pos = 0;
    car.yaw_tar = 0.0f;
    car.lost = 0U;
    car.track_seen = 0U;
}

/**
 * @brief 限幅并输出左右轮 PWM duty
 * @param left 左轮 PWM duty
 * @param right 右轮 PWM duty
 */
static void car_output(int left, int right)
{
    left = car_limit(left, -MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
    right = car_limit(right, -MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
    car.st.duty_l = car_deadzone(left);
    car.st.duty_r = car_deadzone(right);
    Motor_SetDuty(LEFT_MOTOR, (int16_t)car.st.duty_l);
    Motor_SetDuty(RIGHT_MOTOR, (int16_t)car.st.duty_r);
}

/** @brief 输出双轮刹车 */
static void car_brake(void)
{
    car.st.duty_l = 0;
    car.st.duty_r = 0;
    Motor_Brake(LEFT_MOTOR);
    Motor_Brake(RIGHT_MOTOR);
}

/** @brief 从 IMU 缓存更新当前 yaw */
static void car_update_yaw(void)
{
    float ang[3];

    IMU_getYawPitchRoll(ang);
    car.st.yaw = ang[0];
}

/** @brief 使能 IMU 数据就绪中断驱动的姿态更新 */
static void car_prepare_imu(void)
{
    if (car.imu_on == 0U) {
        IMU_init();
        DL_TimerA_stopCounter(TIMER_IMU_TICK_INST);
        NVIC_DisableIRQ(TIMER_IMU_TICK_INST_INT_IRQN);
        DL_GPIO_clearInterruptStatus(GPIO_IMU_INT_PORT, GPIO_IMU_INT_PA16_PIN);
        NVIC_ClearPendingIRQ(GPIO_IMU_INT_INT_IRQN);
        NVIC_EnableIRQ(GPIO_IMU_INT_INT_IRQN);
        car.imu_on = 1U;
    }

    car_update_yaw();
}

/**
 * @brief 根据循迹位图计算黑线位置
 * @param mask 循迹位图
 * @param pos 输出位置
 * @return 检测到黑线的通道数
 */
static uint8_t car_track_pos(uint8_t mask, int *pos)
{
    int sum = 0;
    uint8_t cnt = 0U;

    for (uint8_t i = 0U; i < TRACK_NUM; i++) {
        /* 新模块以位 0 表示该路检测到黑线。 */
        if ((mask & (uint8_t)(1U << i)) == TRACK_STATE_LINE) {
            sum += track_weight[i];
            cnt++;
        }
    }

    if (cnt > 0U) {
        *pos = sum / (int)cnt;
    }
    return cnt;
}

/** @brief 更新循迹模式 */
static void car_update_track(void)
{
    int pos = 0;
    int turn;

    if (car_track_pos(car.st.track_mask, &pos) > 0U) {
        car.st.track_pos = pos;
        car.last_pos = pos;
        car.lost = 0U;
        car.track_seen = 1U;
        turn = (int)PID_CalcTarget(&car.pid_track, 0.0f, (float)pos);
        car_output(car.base - turn, car.base + turn);
        return;
    }

    car.st.track_pos = 0;
    if (car.lost < 255U) {
        car.lost++;
    }

    if ((car.track_find == 0U) ||
        (car.track_seen == 0U) ||
        (car.lost > CAR_TRACK_LOST_MAX)) {
        car_brake();
        return;
    }

    turn = CAR_TRACK_TURN_MAX;
    if (car.last_pos > 0) {
        turn = -CAR_TRACK_TURN_MAX;
    }
    car_output(car.base - turn, car.base + turn);
}

/** @brief 更新定角转向模式 */
static void car_update_turn(void)
{
    float err;
    int turn;

    err = car_wrap_deg(car.yaw_tar - car.st.yaw);
    car.st.done = 0U;
    if (fabsf(err) <= CAR_TURN_DONE_DEG) {
        /* 到达目标角后立即停轮，避免最小 duty 补偿造成继续转动。 */
        car.st.done = 1U;
        car_brake();
        return;
    }
    turn = (int)PID_CalcTarget(&car.pid_yaw, 0.0f, -err);
    car_output(turn, -turn);
}

/** @brief 更新航向保持模式 */
static void car_update_heading(void)
{
    float err;
    int turn;

    err = car_wrap_deg(car.yaw_tar - car.st.yaw);
    turn = (int)PID_CalcTarget(&car.pid_yaw, 0.0f, -err);
    car.st.done = 0U;
    car_output(car.base + turn, car.base - turn);
}

void Car_Init(void)
{
    Encoder_Init();
    Motor_Init();
    PID_Init(&car.pid_track, CAR_TRACK_KP, CAR_TRACK_KI, CAR_TRACK_KD,
             -CAR_TRACK_TURN_MAX, CAR_TRACK_TURN_MAX);
    PID_Init(&car.pid_yaw, CAR_YAW_KP, CAR_YAW_KI, CAR_YAW_KD,
             -CAR_YAW_TURN_MAX, CAR_YAW_TURN_MAX);
    car.imu_on = 0U;
    car.track_find = 1U;
    car.st.track_mask = TRACK_MASK_NO_LINE;
    car.st.track_pos = 0;
    car.st.enc_l = 0;
    car.st.enc_r = 0;
    car.st.duty_l = 0;
    car.st.duty_r = 0;
    car.st.yaw = 0.0f;
    Car_Stop();
    car_prepare_imu();
}

void Car_Stop(void)
{
    PID_Reset(&car.pid_track);
    PID_Reset(&car.pid_yaw);
    car_reset_action();
    car.st.mode = CAR_MODE_STOP;
    car.st.done = 1U;
    car_brake();
}

void Car_SetTrack(int duty)
{
    PID_Reset(&car.pid_track);
    car_reset_action();
    car.base = duty;
    car.st.mode = CAR_MODE_TRACK;
}

void Car_SetTrackLostSearch(uint8_t enable)
{
    car.track_find = 0U;
    if (enable != 0U) {
        car.track_find = 1U;
    }
}

void Car_SetTurnAngle(float deg)
{
    PID_Reset(&car.pid_yaw);
    car_reset_action();
    car_prepare_imu();
    car.yaw_tar = car_wrap_deg(car.st.yaw + deg);
    car.st.mode = CAR_MODE_TURN;
}

void Car_SetHeading(int duty, float yaw)
{
    PID_Reset(&car.pid_yaw);
    car_reset_action();
    car_prepare_imu();
    car.base = duty;
    car.yaw_tar = car_wrap_deg(yaw);
    car.st.mode = CAR_MODE_HEADING;
}

void Car_Update(void)
{
    car.st.enc_l = Encoder_Read(LEFT_ENCODER);
    car.st.enc_r = Encoder_Read(RIGHT_ENCODER);
    car.st.track_mask = Track_ReadMask();
    if (car.imu_on != 0U) {
        car_update_yaw();
    }

    switch (car.st.mode) {
        case CAR_MODE_TRACK:
            car_update_track();
            break;
        case CAR_MODE_TURN:
            car_update_turn();
            break;
        case CAR_MODE_HEADING:
            car_update_heading();
            break;
        case CAR_MODE_STOP:
        default:
            car_brake();
            break;
    }
}

void Car_GetStatus(Car_Status *st)
{
    if (st != 0) {
        *st = car.st;
    }
}

void Car_GetPidParams(Car_PidId id, Car_PidParams *param)
{
    PID const *pid;

    if (param == 0) {
        return;
    }

    pid = &car.pid_track;
    if (id == CAR_PID_YAW) {
        pid = &car.pid_yaw;
    }
    param->kp = pid->kp;
    param->ki = pid->ki;
    param->kd = pid->kd;
}

uint8_t Car_SetPidParam(Car_PidId id, Car_PidTerm term, float value)
{
    PID *pid;

    if ((id != CAR_PID_TRACK && id != CAR_PID_YAW) ||
        (term != CAR_PID_KP && term != CAR_PID_KI && term != CAR_PID_KD) ||
        !isfinite(value)) {
        return 0U;
    }

    pid = &car.pid_track;
    if (id == CAR_PID_YAW) {
        pid = &car.pid_yaw;
    }
    if (term == CAR_PID_KP) {
        pid->kp = value;
    } else if (term == CAR_PID_KI) {
        pid->ki = value;
    } else {
        pid->kd = value;
    }

    PID_Reset(pid);
    return 1U;
}
