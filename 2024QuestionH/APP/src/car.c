/**
 * @file car.c
 * @brief 小车运动控制层实现
 */
#include "car.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "pid.h"

/* 速度环采用“10 倍误差映射 + 增量式 PI 修正”。 */
#define CAR_KP          0.0f
#define CAR_KI          0.2f
#define CAR_KD          0.0f
#define CAR_INC_LIMIT   50.0f
#define CAR_ERR_GAIN    10

typedef struct {
    PID pid_l;
    PID pid_r;
    int target_l;
    int target_r;
    int speed_l;
    int speed_r;
    int duty_l;
    int duty_r;
} Car_State;

static Car_State car;

/**
 * @brief 将整数限制到闭区间
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的整数
 */
static int car_limit_int(int val, int min, int max)
{
    if (val > max) {
        return max;
    }

    if (val < min) {
        return min;
    }

    return val;
}

void Car_Init(void)
{
    encoder_init();
    Motor_Init();

    PID_Init(&car.pid_l, CAR_KP, CAR_KI, CAR_KD,
             -CAR_INC_LIMIT, CAR_INC_LIMIT);
    PID_Init(&car.pid_r, CAR_KP, CAR_KI, CAR_KD,
             -CAR_INC_LIMIT, CAR_INC_LIMIT);
    Car_Stop();
}

void Car_Stop(void)
{
    car.target_l = 0;
    car.target_r = 0;
    car.duty_l = 0;
    car.duty_r = 0;

    PID_Reset(&car.pid_l);
    PID_Reset(&car.pid_r);
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
}

void Car_SetWheelSpeed(int left, int right)
{
    car.target_l = left;
    car.target_r = right;
}

void Car_SetSpeed(int linear, int turn)
{
    Car_SetWheelSpeed(linear - turn, linear + turn);
}

void Car_Update(void)
{
    int inc_l;
    int inc_r;
    int err_l;
    int err_r;

    car.speed_l = encoder_get_count(ENCODER_2);
    car.speed_r = encoder_get_count(ENCODER_1);

    if ((car.target_l == 0) && (car.target_r == 0)) {
        Car_Stop();
        return;
    }

    err_l = car.target_l - car.speed_l;
    err_r = car.target_r - car.speed_r;
    inc_l = err_l * CAR_ERR_GAIN +
            (int)PID_CalcIncTarget(&car.pid_l, (float)car.target_l, (float)car.speed_l);
    inc_r = err_r * CAR_ERR_GAIN +
            (int)PID_CalcIncTarget(&car.pid_r, (float)car.target_r, (float)car.speed_r);

    car.duty_l = car_limit_int(car.duty_l + inc_l,
                               -MOTOR_PWM_PERIOD,
                               MOTOR_PWM_PERIOD);
    car.duty_r = car_limit_int(car.duty_r + inc_r,
                               -MOTOR_PWM_PERIOD,
                               MOTOR_PWM_PERIOD);

    Motor_SetDuty(MOTOR_B, (int16_t)car.duty_l);
    Motor_SetDuty(MOTOR_A, (int16_t)car.duty_r);
}

int Car_GetTargetLeft(void)
{
    return car.target_l;
}

int Car_GetTargetRight(void)
{
    return car.target_r;
}

int Car_GetSpeedLeft(void)
{
    return car.speed_l;
}

int Car_GetSpeedRight(void)
{
    return car.speed_r;
}

int Car_GetDutyLeft(void)
{
    return car.duty_l;
}

int Car_GetDutyRight(void)
{
    return car.duty_r;
}
