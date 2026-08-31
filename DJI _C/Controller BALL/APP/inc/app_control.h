/**
 * @file app_control.h
 * @brief 视觉到电机的平衡控制框架接口。
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_pid.h"
#include "app_vision.h"
#include "bsp_motor.h"

#define APP_CONTROL_RX_RING_SIZE 128U
#define APP_CONTROL_FRAME_SIZE   64U

/** @brief 平衡控制状态。 */
typedef enum
{
    APP_CONTROL_DISABLED = 0,
    APP_CONTROL_STARTUP,
    APP_CONTROL_READY,
    APP_CONTROL_BALANCE,
    APP_CONTROL_FAULT
} app_control_state_t;

/** @brief 平衡控制框架错误。 */
typedef enum
{
    APP_CONTROL_OK = 0,
    APP_CONTROL_BUSY,
    APP_CONTROL_ERR_ARG,
    APP_CONTROL_ERR_RANGE,
    APP_CONTROL_ERR_STATE,
    APP_CONTROL_ERR_PID,
    APP_CONTROL_ERR_MOTOR
} app_control_err_t;

/** @brief 当前等待完成的电机操作。 */
typedef enum
{
    APP_CONTROL_MOTOR_NONE = 0,
    APP_CONTROL_MOTOR_ENABLE,
    APP_CONTROL_MOTOR_DISABLE,
    APP_CONTROL_MOTOR_ANGLE
} app_control_motor_op_t;

/** @brief 视觉、PID 和电机之间的控制上下文。 */
typedef struct
{
    bsp_motor_t *motor;
    app_pid_t *pid;
    bsp_motor_fb_t motor_fb;
    bsp_motor_err_t motor_err;
    app_control_state_t state;
    app_control_motor_op_t motor_op;
    float target_pos;
    float motor_dir;
    float angle_max;
    float position_pos;
    /** @brief Latest received vision speed in pixels per second, retained for observation. */
    float speed_pos;
    /** @brief Vision-speed multiplier for gated PID derivative feedback. */
    float speed_d_scale;
    uint32_t ctrl_period_ms;
    uint32_t vision_timeout_ms;
    uint32_t last_vision_ms;
    uint32_t last_ctrl_ms;
    uint32_t frame_seq;
    uint8_t frame_seen;
    uint8_t vision_valid;
    uint8_t motor_enabled;
    uint8_t stop_requested;
    /** @brief Nonzero when a vision timing-end marker is waiting to be handled. */
    uint8_t timing_end;
    volatile uint8_t rx_overflow;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint16_t frame_len;
    uint8_t rx_ring[APP_CONTROL_RX_RING_SIZE];
    uint8_t frame[APP_CONTROL_FRAME_SIZE];
} app_control_t;

/**
 * @brief 初始化平衡控制上下文。
 * @param ctl 待初始化的控制上下文。
 * @param motor 已初始化的 QD4310 电机上下文。
 * @param pid 已初始化的 PID 上下文。
 * @param target_pos 视觉位置目标，单位由视觉协议标定确定。
 * @param motor_dir 电机方向符号，只允许为 1.0 或 -1.0。
 * @param angle_max 相对零点的最大目标角度，单位为 rad，范围为 0～pi。
 * @param ctrl_period_ms 控制周期，单位为 ms。
 * @param vision_timeout_ms 视觉数据超时阈值，单位为 ms。
 * @return 成功返回 APP_CONTROL_OK，否则返回参数或范围错误。
 */
app_control_err_t APP_Control_Init(app_control_t *ctl, bsp_motor_t *motor,
                                   app_pid_t *pid, float target_pos,
                                   float motor_dir, float angle_max,
                                   uint32_t ctrl_period_ms,
                                   uint32_t vision_timeout_ms);

/**
 * @brief 启动上电归零序列，或复用已经使能的电机进入就绪状态。
 * @param ctl 已初始化的控制上下文。
 * @return 成功启动返回 APP_CONTROL_OK，已有事务返回 APP_CONTROL_BUSY。
 */
app_control_err_t APP_Control_Start(app_control_t *ctl);

/**
 * @brief Adopt a motor enable state completed by another task.
 * @param ctl Visual balance controller context.
 * @param enabled Nonzero when the shared motor is enabled.
 */
void APP_Control_AdoptMotorState(app_control_t *ctl, uint8_t enabled);

/**
 * @brief Configure vision-speed feedback into the PID derivative output.
 * @param ctl Visual balance controller context.
 * @param scale Multiplier applied to vision speed; zero disables feedback.
 */
void APP_Control_SetSpeedDScale(app_control_t *ctl, float scale);

/**
 * @brief 请求视觉控制器在当前电机事务完成后异步失能并退出。
 * @param ctl 已初始化的控制上下文。
 * @return 成功记录请求返回 APP_CONTROL_OK，否则返回参数或状态错误。
 */
app_control_err_t APP_Control_Stop(app_control_t *ctl);

/**
 * @brief 在串口接收中断中压入一个字节。
 * @param ctl 控制上下文。
 * @param byte 新接收的字节。
 * @note 该接口只执行整数环形缓冲操作，不能在其中执行解析、浮点或电机事务。
 */
void APP_Control_RxByte(app_control_t *ctl, uint8_t byte);

/**
 * @brief Take and clear the vision timing-end marker event.
 * @param ctl Visual balance controller context.
 * @return 1 when a timing-end marker was received, otherwise 0.
 */
uint8_t APP_Control_TakeTimingEnd(app_control_t *ctl);

/**
 * @brief 在主循环中处理已接收的视觉字节并解析完整帧。
 * @param ctl 控制上下文。
 * @param now_ms 当前 HAL tick，单位为 ms。
 * @return 成功返回 APP_CONTROL_OK，否则返回参数错误。
 * @note 视觉帧格式为 AA + 可选负号 + 像素数字 + FF + '\n'。
 */
app_control_err_t APP_Control_ProcessRx(app_control_t *ctl,
                                        uint32_t now_ms);

/**
 * @brief 异步启动电机使能或失能事务。
 * @param ctl 控制上下文。
 * @param enable 非零表示使能，0 表示失能。
 * @return 成功启动返回 APP_CONTROL_OK，已有事务返回 APP_CONTROL_BUSY。
 */
app_control_err_t APP_Control_SetEnabled(app_control_t *ctl, uint8_t enable);

/**
 * @brief 执行一次控制调度。
 * @param ctl 控制上下文。
 * @param now_ms 当前 HAL tick，单位为 ms。
 * @return 成功返回 APP_CONTROL_OK，否则返回状态、PID 或电机错误。
 * @note 未到控制周期、视觉数据无效或视觉数据超时时不会发送运动命令。
 */
app_control_err_t APP_Control_Tick(app_control_t *ctl, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */
