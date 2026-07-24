/**
 * @file car.c
 * @brief 通用底盘控制层实现
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
// 速度环参数
#define CAR_SPD_KP              90.0f
#define CAR_SPD_KI              8.0f
#define CAR_SPD_KD              0.0f
// 循迹环参数
#define CAR_TRACK_KP            0.05f
#define CAR_TRACK_KI            0.0f
#define CAR_TRACK_KD            0.0f
#define CAR_TRACK_TURN_MAX      18
#define CAR_TRACK_SEARCH_BASE   8
#define CAR_TRACK_LOST_MAX      10U
#define CAR_TRACK_SLOW_DIV      300
// 角度换参数
#define CAR_YAW_KP              2.0f
#define CAR_YAW_KI              0.0f
#define CAR_YAW_KD              0.0f
#define CAR_YAW_TURN_MAX        18
#define CAR_TURN_DONE_DEG       2.0f

static const int track_weight[TRACK_NUM] = {
    -2000, -1100, -600, -100, 100, 600, 1100, 2000
};

typedef struct {
    Car_Status st;
    int base_set;
    int last_pos;
    uint8_t lost;
    uint8_t imu_on;
    PID pid_spd_l;
    PID pid_spd_r;
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

/**
 * @brief 获取整数绝对值
 * @param val 输入值
 * @return 绝对值
 */
static int car_abs_int(int val)
{
    if (val < 0) {
        return -val;
    }

    return val;
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

/**
 * @brief 计算目标角与当前角的最短差值
 * @param tar 目标角，单位为度
 * @param now 当前角，单位为度
 * @return 最短角差，单位为度
 */
static float car_angle_err(float tar, float now)
{
    return car_wrap_deg(tar - now);
}

/**
 * @brief 清空底盘状态量
 */
static void car_clear_state(void)
{
    car.st.done = 0U;
    car.st.track_mask = 0U;
    car.st.track_pos = 0;
    car.st.base = 0;
    car.st.turn = 0;
    car.st.target_l = 0;
    car.st.target_r = 0;
    car.st.speed_l = 0;
    car.st.speed_r = 0;
    car.st.duty_l = 0;
    car.st.duty_r = 0;
    car.st.yaw = 0.0f;
    car.st.yaw_tar = 0.0f;
    car.base_set = 0;
    car.last_pos = 0;
    car.lost = 0U;
}

/**
 * @brief 重置所有控制器内部状态
 */
static void car_reset_pid(void)
{
    PID_Reset(&car.pid_spd_l);
    PID_Reset(&car.pid_spd_r);
    PID_Reset(&car.pid_track);
    PID_Reset(&car.pid_yaw);
}

/**
 * @brief 将双轮目标速度写入状态
 * @param left 左轮目标速度
 * @param right 右轮目标速度
 */
static void car_set_wheel_target(int left, int right)
{
    car.st.target_l = left;
    car.st.target_r = right;
    car.st.base = (left + right) / 2;
    car.st.turn = (right - left) / 2;
}

/**
 * @brief 根据基础速度和差速生成双轮目标速度
 * @param base 基础速度
 * @param turn 差速转向量
 */
static void car_set_base_turn(int base, int turn)
{
    car.st.base = base;
    car.st.turn = turn;
    car.st.target_l = base - turn;
    car.st.target_r = base + turn;
}

/**
 * @brief 输出双轮停止命令
 */
static void car_apply_stop(void)
{
    car.st.target_l = 0;
    car.st.target_r = 0;
    car.st.base = 0;
    car.st.turn = 0;
    car.st.duty_l = 0;
    car.st.duty_r = 0;
    Motor_SetDuty(LEFT_MOTOR, 0);
    Motor_SetDuty(RIGHT_MOTOR, 0);
}

/**
 * @brief 更新双轮编码器反馈
 */
static void car_update_speed(void)
{
    car.st.speed_l = Encoder_Read(LEFT_ENCODER);
    car.st.speed_r = Encoder_Read(RIGHT_ENCODER);
}

/**
 * @brief 更新当前航向角
 */
static void car_update_yaw(void)
{
    float ang[3];

    IMU_getYawPitchRoll(ang);
    car.st.yaw = ang[0];
}

/**
 * @brief 使能 IMU 姿态更新链路
 */
static void car_prepare_imu(void)
{
    if (car.imu_on == 0U) {
        IMU_init();
        // 切换到由 IMU 数据就绪中断驱动姿态更新。
        DL_TimerA_stopCounter(TIMER_IMU_TICK_INST);
        NVIC_DisableIRQ(TIMER_IMU_TICK_INST_INT_IRQN);
        DL_GPIO_clearInterruptStatus(GPIO_IMU_INT_PORT, GPIO_IMU_INT_PA16_PIN);
        NVIC_ClearPendingIRQ(GPIO_IMU_INT_INT_IRQN);
        NVIC_EnableIRQ(GPIO_IMU_INT_INT_IRQN);
        car.imu_on = 1U;
    }

    IMU_resetTimestamp();
    car_update_yaw();
}

/**
 * @brief 将速度环输出作用到左右电机
 */
static void car_apply_speed_pid(void)
{
    // 将速度误差转换成左右轮 PWM 占空比。
    car.st.duty_l = car_limit_int((int)PID_CalcTarget(&car.pid_spd_l,
                                                      (float)car.st.target_l,
                                                      (float)car.st.speed_l),
                                  -MOTOR_PWM_PERIOD,
                                  MOTOR_PWM_PERIOD);
    car.st.duty_r = car_limit_int((int)PID_CalcTarget(&car.pid_spd_r,
                                                      (float)car.st.target_r,
                                                      (float)car.st.speed_r),
                                  -MOTOR_PWM_PERIOD,
                                  MOTOR_PWM_PERIOD);

    Motor_SetDuty(LEFT_MOTOR, (int16_t)car.st.duty_l);
    Motor_SetDuty(RIGHT_MOTOR, (int16_t)car.st.duty_r);
}

/**
 * @brief 根据循迹位图计算黑线加权位置
 * @param mask 循迹位图
 * @param pos 输出黑线位置
 * @return 检测到黑线的通道数
 */
static uint8_t car_calc_track_pos(uint8_t mask, int *pos)
{
    int sum = 0;
    uint8_t cnt = 0U;

    // 按通道权重求平均，得到黑线相对车体中心的位置。
    for (uint8_t i = 0U; i < TRACK_NUM; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            sum += track_weight[i];
            cnt++;
        }
    }

    if (cnt > 0U) {
        *pos = sum / (int)cnt;
    }

    return cnt;
}

/**
 * @brief 更新速度模式目标
 */
static void car_update_mode_speed(void)
{
    car.st.done = 0U;
}

/**
 * @brief 更新循迹模式目标
 */
static void car_update_mode_track(void)
{
    uint8_t cnt;
    int pos = 0;
    int turn;
    int base;

    car.st.track_mask = Track_ReadMask();
    cnt = car_calc_track_pos(car.st.track_mask, &pos);

    if (cnt > 0U) {
        // 检测到黑线时，按偏差减速并生成差速转向量。
        car.st.track_pos = pos;
        car.last_pos = pos;
        car.lost = 0U;

        base = car.base_set - car_abs_int(pos) / CAR_TRACK_SLOW_DIV;
        if (base < CAR_TRACK_SEARCH_BASE) {
            base = CAR_TRACK_SEARCH_BASE;
        }

        turn = (int)PID_CalcTarget(&car.pid_track, 0.0f, (float)pos);
        turn = car_limit_int(turn, -CAR_TRACK_TURN_MAX, CAR_TRACK_TURN_MAX);
        car_set_base_turn(base, turn);
        return;
    }

    car.st.track_pos = 0;
    if (car.lost < 255U) {
        car.lost++;
    }

    if (car.lost > CAR_TRACK_LOST_MAX) {
        // 连续丢线超阈值后直接停车，避免继续盲搜。
        car_set_base_turn(0, 0);
        return;
    }

    // 短时丢线时沿上次偏移方向低速搜索。
    base = car.base_set;
    if (base > CAR_TRACK_SEARCH_BASE) {
        base = CAR_TRACK_SEARCH_BASE;
    }

    turn = CAR_TRACK_TURN_MAX;
    if (car.last_pos > 0) {
        turn = -CAR_TRACK_TURN_MAX;
    }

    car_set_base_turn(base, turn);
}

/**
 * @brief 更新定角转向模式目标
 */
static void car_update_mode_turn(void)
{
    float err;
    int turn;

    car_update_yaw();
    err = car_angle_err(car.st.yaw_tar, car.st.yaw);
    if (fabsf(err) <= CAR_TURN_DONE_DEG) {
        // 角度进入容差后立即置完成并停轮。
        car.st.done = 1U;
        car_apply_stop();
        return;
    }

    // 用航向误差生成原地转向差速。
    turn = (int)PID_CalcTarget(&car.pid_yaw, 0.0f, -err);
    turn = car_limit_int(turn, -CAR_YAW_TURN_MAX, CAR_YAW_TURN_MAX);
    car.st.done = 0U;
    car_set_base_turn(0, turn);
}

/**
 * @brief 更新航向保持模式目标
 */
static void car_update_mode_heading(void)
{
    float err;
    int turn;

    car_update_yaw();
    err = car_angle_err(car.st.yaw_tar, car.st.yaw);
    // 保持基础前进速度，仅叠加航向修正量。
    turn = (int)PID_CalcTarget(&car.pid_yaw, 0.0f, -err);
    turn = car_limit_int(turn, -CAR_YAW_TURN_MAX, CAR_YAW_TURN_MAX);
    car.st.done = 0U;
    car_set_base_turn(car.base_set, turn);
}

void Car_Init(void)
{
    Encoder_Init();
    Motor_Init();

    PID_Init(&car.pid_spd_l, CAR_SPD_KP, CAR_SPD_KI, CAR_SPD_KD,
             -MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
    PID_Init(&car.pid_spd_r, CAR_SPD_KP, CAR_SPD_KI, CAR_SPD_KD,
             -MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
    PID_Init(&car.pid_track, CAR_TRACK_KP, CAR_TRACK_KI, CAR_TRACK_KD,
             -CAR_TRACK_TURN_MAX, CAR_TRACK_TURN_MAX);
    PID_Init(&car.pid_yaw, CAR_YAW_KP, CAR_YAW_KI, CAR_YAW_KD,
             -CAR_YAW_TURN_MAX, CAR_YAW_TURN_MAX);

    car.imu_on = 0U;
    car_clear_state();
    Car_Stop();
}

void Car_Stop(void)
{
    car_reset_pid();
    car_clear_state();
    car.st.mode = CAR_MODE_STOP;
    car.st.done = 1U;
    car_apply_stop();
}

void Car_SetMode(Car_Mode mode)
{
    if (mode == CAR_MODE_STOP) {
        Car_Stop();
        return;
    }

    car.st.mode = mode;
    car.st.done = 0U;
}

void Car_SetWheelSpeed(int left, int right)
{
    Car_SetMode(CAR_MODE_SPEED);
    car_set_wheel_target(left, right);
}

void Car_SetSpeed(int base, int turn)
{
    Car_SetMode(CAR_MODE_SPEED);
    car_set_base_turn(base, turn);
}

void Car_SetTrack(int base)
{
    car_reset_pid();
    car_clear_state();
    // 记录巡线基础速度，由巡线环在运行时动态修正。
    car.base_set = base;
    car.st.mode = CAR_MODE_TRACK;
}

void Car_SetTurnAngle(float deg)
{
    car_reset_pid();
    car_clear_state();
    car_prepare_imu();
    // 以当前航向为基准生成相对转角目标。
    car.st.mode = CAR_MODE_TURN;
    car.st.yaw_tar = car_wrap_deg(car.st.yaw + deg);
}

void Car_SetHeading(int base, float yaw)
{
    car_reset_pid();
    car_clear_state();
    car_prepare_imu();
    // 锁定绝对航向，同时保留给定巡航速度。
    car.base_set = base;
    car.st.mode = CAR_MODE_HEADING;
    car.st.yaw_tar = car_wrap_deg(yaw);
}

void Car_Update(void)
{
    car_update_speed();

    switch (car.st.mode) {
        case CAR_MODE_SPEED:
            // 纯速度模式只维护双轮目标速度。
            car_update_mode_speed();
            car_apply_speed_pid();
            break;

        case CAR_MODE_TRACK:
            // 巡线模式先更新目标，再走统一速度闭环。
            car_update_mode_track();
            car_apply_speed_pid();
            break;

        case CAR_MODE_TURN:
            car_update_mode_turn();
            if (car.st.done == 0U) {
                // 未到位前持续输出原地转向控制量。
                car_apply_speed_pid();
            }
            break;

        case CAR_MODE_HEADING:
            car_update_mode_heading();
            car_apply_speed_pid();
            break;

        case CAR_MODE_STOP:
        default:
            car_apply_stop();
            break;
    }
}

uint8_t Car_IsDone(void)
{
    return car.st.done;
}

void Car_GetStatus(Car_Status *st)
{
    if (st == 0) {
        return;
    }

    *st = car.st;
}

void Car_GetPidParam(Car_PidParam *param)
{
    PID *pid;

    if (param == 0) {
        return;
    }

    if (car.st.mode == CAR_MODE_TRACK) {
        pid = &car.pid_track;
    } else if ((car.st.mode == CAR_MODE_TURN) ||
               (car.st.mode == CAR_MODE_HEADING)) {
        pid = &car.pid_yaw;
    } else {
        pid = &car.pid_spd_l;
    }

    param->kp = pid->kp;
    param->ki = pid->ki;
    param->kd = pid->kd;
}
