/**
 * @file car.c
 * @brief 小车运动控制层实现
 */
#include "car.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "pid.h"

#define CAR_KP          1.0f
#define CAR_KI          0.0f
#define CAR_KD          0.0f

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
 * @brief 将 PID 浮点输出转换为电机占空比整数
 * @param out PID 输出
 * @return 电机占空比值
 */
static int car_out_to_duty(float out)
{
    if (out > (float)MOTOR_PWM_PERIOD) {
        return MOTOR_PWM_PERIOD;
    }

    if (out < (float)(-MOTOR_PWM_PERIOD)) {
        return -MOTOR_PWM_PERIOD;
    }

    return (int)out;
}

void Car_Init(void)
{
    encoder_init();
    Motor_Init();

    PID_Init(&car.pid_l, CAR_KP, CAR_KI, CAR_KD,
             (float)(-MOTOR_PWM_PERIOD), (float)MOTOR_PWM_PERIOD);
    PID_Init(&car.pid_r, CAR_KP, CAR_KI, CAR_KD,
             (float)(-MOTOR_PWM_PERIOD), (float)MOTOR_PWM_PERIOD);
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
    float out_l;
    float out_r;

    car.speed_l = encoder_get_count(ENCODER_1);
    car.speed_r = encoder_get_count(ENCODER_2);

    if ((car.target_l == 0) && (car.target_r == 0)) {
        Car_Stop();
        return;
    }

    out_l = PID_CalcTarget(&car.pid_l, (float)car.target_l, (float)car.speed_l);
    out_r = PID_CalcTarget(&car.pid_r, (float)car.target_r, (float)car.speed_r);
    car.duty_l = car_out_to_duty(out_l);
    car.duty_r = car_out_to_duty(out_r);

    Motor_SetDuty(MOTOR_A, (int16_t)car.duty_l);
    Motor_SetDuty(MOTOR_B, (int16_t)car.duty_r);
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
