/**
 * @file pid_test.c
 * @brief 单轮 PID 控速整定测试实现
 */

#include "pid_test.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "pid.h"
#include "ui.h"

/* 与 car.c 保持一致：10 倍误差映射 + 增量式 PI 修正。 */
#define PID_TEST_KP         1.0f
#define PID_TEST_KI         0.1f
#define PID_TEST_KD         0.0f
#define PID_TEST_INC_LIMIT  50.0f
#define PID_TEST_ERR_GAIN   100

#define PID_TEST_STEP       5
#define PID_TEST_MAX        40
#define PID_TEST_MIN        -40

#define PID_TEST_L_ENC      ENCODER_E2
#define PID_TEST_R_ENC      ENCODER_E1
#define PID_TEST_L_MOTOR    MOTOR_B
#define PID_TEST_R_MOTOR    MOTOR_A

/**
 * @brief 单次按键动作后等待所有按键释放
 */
static void pid_test_wait_key_release(void)
{
    while (Key_Scan() != -1) {
        delay_ms(10);
    }
}

/**
 * @brief 将整数限制到闭区间
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的整数
 */
static int pid_test_limit_int(int val, int min, int max)
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
 * @brief 将编码器 ID 转成调参界面使用的轮子名称
 * @param id 编码器选择
 * @return 轮子名称
 */
static const char *pid_test_wheel_name(Encoder_ID id)
{
    return (id == PID_TEST_L_ENC) ? "Left" : "Right";
}

/**
 * @brief 将编码器 ID 映射为对应电机
 * @param id 编码器选择
 * @return 电机选择
 */
static Motor_ID pid_test_motor_of(Encoder_ID id)
{
    return (id == PID_TEST_L_ENC) ? PID_TEST_L_MOTOR : PID_TEST_R_MOTOR;
}

/**
 * @brief 选择需要整定的轮子
 * @return 返回被选择的编码器 ID
 */
static Encoder_ID pid_test_select_wheel(void)
{
    uint8_t left = 1U;
    int8_t key;

    while (1) {
        // 先在界面上显示当前待调的轮子。
        UI_Test_PIDSelect(left);

        do {
            key = Key_Scan();
        } while (key == -1);

        pid_test_wait_key_release();

        if (key == KEY_1 || key == KEY_2) {
            // 左右键共用切换逻辑，方便单手调试。
            left ^= 1U;
        } else if (key == KEY_3) {
            return left ? PID_TEST_L_ENC : PID_TEST_R_ENC;
        }
    }
}

void PID_Test_Run(void)
{
    PID pid_l;
    PID pid_r;
    PID *pid;
    Encoder_ID enc_id;
    Motor_ID motor_id;
    Motor_ID motor_off;
    const char *wheel;
    int target = 0;
    int actual = 0;
    int err = 0;
    int inc = 0;
    int out = 0;
    int8_t key;

    UI_Init();

    enc_id = pid_test_select_wheel();
    motor_id = pid_test_motor_of(enc_id);
    motor_off = (motor_id == MOTOR_A) ? MOTOR_B : MOTOR_A;
    wheel = pid_test_wheel_name(enc_id);

    // 只给被调轮建立 PID，另一侧电机保持关闭。
    Encoder_Init();
    Motor_Init();
    PID_Init(&pid_l, PID_TEST_KP, PID_TEST_KI, PID_TEST_KD,
             -PID_TEST_INC_LIMIT, PID_TEST_INC_LIMIT);
    PID_Init(&pid_r, PID_TEST_KP, PID_TEST_KI, PID_TEST_KD,
             -PID_TEST_INC_LIMIT, PID_TEST_INC_LIMIT);

    pid = (enc_id == PID_TEST_L_ENC) ? &pid_l : &pid_r;
    Motor_SetDuty(motor_off, 0);
    Motor_SetDuty(motor_id, 0);

    UI_Test_PID(wheel, actual, target, out);

    while (1) {
        key = Key_Scan();
        if (key != -1) {
            if (key == KEY_1) {
                target += PID_TEST_STEP;
            } else if (key == KEY_2) {
                target -= PID_TEST_STEP;
            } else if (key == KEY_3) {
                target = 0;
                PID_Reset(pid);
                Motor_SetDuty(motor_id, 0);
            }

            target = pid_test_limit_int(target, PID_TEST_MIN, PID_TEST_MAX);
            pid_test_wait_key_release();
        }

        // 编码器实时速度作为本轮 PID 的反馈量。
        actual = Encoder_Read(enc_id);
        err = target - actual;
        if (target == 0) {
            // 目标为零时清空调节量，避免积分残留拖动电机。
            err = 0;
            inc = 0;
            out = 0;
            Motor_SetDuty(motor_id, 0);
        } else {
            // 将经验前馈与增量式 PID 叠加，再累计成最终输出。
            inc = err * PID_TEST_ERR_GAIN +
                  (int)PID_CalcIncTarget(pid, (float)target, (float)actual);
            out += inc;
            out = pid_test_limit_int(out, -MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
            Motor_SetDuty(motor_id, (int16_t)out);
        }

        // 未参与调参的另一侧电机始终拉低，避免串扰。
        Motor_SetDuty(motor_off, 0);

        UI_Test_PID(wheel, actual, target, out);
        lc_printf("actual,target,err,inc,out:%d , %d , %d , %d , %d\r\n",
                  actual,
                  target,
                  err,
                  inc,
                  out);
        delay_ms(50);
    }
}
