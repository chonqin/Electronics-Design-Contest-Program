# 平衡杆小球控制链路与 API 指南

本文档依据当前工程源码整理，用于理解平衡杆小球系统从视觉测量到电机动作的完整控制链路，并作为后续 PID 调参参考。

当前系统的核心结构是：

> 视觉位置外环 PID → 计算电机绝对目标角度 → 电机驱动器内部完成角度控制。

STM32 上的 PID 不直接控制电机电流或速度，也没有使用电机反馈角度参与 PID。STM32 根据小球位置计算杆的目标倾角，然后通过绝对角度命令交给 QD4310 执行。

当前 PID 参数 Kp、Ki、Kd 都是 0，因此控制链路虽然会运行，但 PID 输出始终为 0 rad。电机上电后会执行一次零角度命令，不会根据小球位置产生非零倾角。

---

## 1. 工程分层

当前控制代码分为四层：

| 层级 | 主要文件 | 作用 |
|---|---|---|
| CubeMX/HAL 层 | Src/main.c、Src/usart.c、Src/dma.c | 初始化串口和 DMA，提供中断与 DMA 回调 |
| BSP 电机层 | BSP/inc/bsp_motor.h、BSP/src/bsp_motor.c | 编码 QD4310 命令、启动 DMA、解析电机反馈 |
| APP 算法层 | APP/inc/app_pid.h、APP/src/app_pid.c | PID 运算、积分限幅和输出限幅 |
| APP 控制层 | APP/inc/app_control.h、APP/src/app_control.c | 视觉接收、控制状态机、周期调度和角度限位 |

视觉协议解析位于：

- APP/inc/app_vision.h
- APP/src/app_vision.c

---

## 2. 整体控制链路

~~~mermaid
flowchart LR
    A["视觉模块<br/>GB,seq,valid,position"] --> B["USART1<br/>单字节接收中断"]
    B --> C["APP_Control_RxByte<br/>环形缓冲区"]
    C --> D["APP_Control_ProcessRx<br/>拼接完整文本帧"]
    D --> E["APP_Vision_Parse<br/>原始值换算为 cm"]
    E --> F["position_cm<br/>小球当前位置"]

    F --> G["APP_Control_Tick<br/>约 17 ms 调度"]
    H["target_cm<br/>目标位置"] --> G
    G --> I["APP_PID_Update<br/>位置 PID"]
    I --> J["motor_dir<br/>方向修正"]
    J --> K["±0.45 rad<br/>机械角度限幅"]
    K --> L["负角映射到 0～2π"]
    L --> M["BSP_MOTOR_SetAngle<br/>0x05 绝对角度模式"]

    M --> N["USART6 TX DMA<br/>发送 5 字节"]
    O["USART6 RX DMA<br/>接收 10 字节反馈"] --> P["BSP_MOTOR_Process"]
    N --> Q["QD4310 内部角度控制"]
    Q --> O
    Q --> R["平衡杆倾斜"]
    R --> S["小球运动"]
    S --> A

    P -. "只用于通信确认和监控<br/>不进入当前 PID" .-> G
~~~

一次正常控制的数据路径是：

~~~text
视觉位置
→ USART1 中断接收
→ 环形缓冲区
→ 视觉帧解析
→ 得到 position_cm
→ 判断数据是否有效、是否超时
→ 按 17 ms 周期执行 PID
→ 输出相对零点的有符号角度
→ 方向修正
→ 限制在 -0.45～+0.45 rad
→ 映射为 0～2π 的绝对角度
→ USART6 DMA 发送给电机
→ 等待电机 10 字节反馈
→ 反馈合法后才允许下一次控制
~~~

---

## 3. 上电启动链路

### 3.1 HAL 和外设初始化

main() 依次执行：

~~~c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_DMA_Init();
MX_USART1_UART_Init();
MX_USART6_UART_Init();
~~~

当前源码中的 USART1 和 USART6 都配置为：

~~~text
波特率：115200
数据位：8
校验位：无
停止位：1
硬件流控：无
~~~

二者的接收方式不同：

- USART1 使用单字节接收中断，负责视觉文本帧。
- USART6 使用 TX DMA 和 RX DMA，负责 QD4310 二进制协议。
- USART6 的 TX/RX DMA 都是 DMA_NORMAL 模式，每条命令重新启动一次 DMA。

### 3.2 初始化电机 BSP

~~~c
BSP_MOTOR_Init(&motor, &huart6, MOTOR_ID, MOTOR_TIMEOUT_MS);
~~~

当前参数：

~~~text
电机 ID         = 0x01
单次事务超时    = 100 ms
~~~

该函数只初始化软件上下文并检查：

- 电机上下文和 UART 句柄是否有效；
- 电机 ID 是否在 0x00～0x0F；
- USART6 是否同时存在 TX/RX DMA；
- TX/RX DMA 是否均为 DMA_NORMAL；
- 事务超时是否大于 0。

该函数不会使能电机，也不会发送串口数据。

### 3.3 初始化 PID

~~~c
APP_PID_Init(&balance_pid,
             BALANCE_PID_KP,
             BALANCE_PID_KI,
             BALANCE_PID_KD,
             -BALANCE_ANGLE_LIMIT_RAD,
             BALANCE_ANGLE_LIMIT_RAD,
             -BALANCE_PID_I_LIMIT,
             BALANCE_PID_I_LIMIT);
~~~

当前参数：

~~~text
Kp = 0
Ki = 0
Kd = 0

PID 输出范围 = -0.45～+0.45 rad
积分状态范围 = -1～+1
~~~

BALANCE_PID_I_LIMIT 限制的是积分状态，也就是误差对时间的积分，不是最终积分输出。

### 3.4 初始化应用控制状态机

~~~c
APP_Control_Init(&control,
                 &motor,
                 &balance_pid,
                 BALANCE_TARGET_CM,
                 BALANCE_MOTOR_DIR,
                 BALANCE_ANGLE_LIMIT_RAD,
                 CONTROL_PERIOD_MS,
                 VISION_TIMEOUT_MS);
~~~

当前参数：

~~~text
目标位置 target_cm       = 0 cm
电机方向 motor_dir       = +1
最大相对角度 angle_max   = 0.45 rad
控制周期                 = 17 ms
视觉超时                 = 100 ms
~~~

初始化后：

~~~text
control.state         = APP_CONTROL_DISABLED
control.motor_enabled = 0
control.motor_op      = APP_CONTROL_MOTOR_NONE
~~~

### 3.5 启动视觉串口接收

~~~c
HAL_UART_Receive_IT(&huart1, &vision_rx_byte, 1U);
~~~

该函数让 USART1 等待一个字节。每收到一个字节就会进入 HAL_UART_RxCpltCallback()。

回调完成两件事：

1. 调用 APP_Control_RxByte() 把字节压入环形缓冲区。
2. 再次调用 HAL_UART_Receive_IT()，接收下一个字节。

### 3.6 自动使能和零角度命令

初始化成功后调用：

~~~c
APP_Control_Start(&control);
~~~

启动过程是异步状态机：

~~~text
APP_Control_Start
→ BSP_MOTOR_Enable
→ USART6 RX DMA 启动
→ USART6 TX DMA 发送使能命令
→ 等待 TX/RX DMA 回调
→ APP_Control_Tick 调用 BSP_MOTOR_Process
→ 检查反馈 ID、CRC 和 enabled
→ BSP_MOTOR_SetAngle(0.0 rad)
→ 再执行一次 DMA 一发一收
→ 收到合法反馈
→ APP_CONTROL_READY
~~~

这里发送的是绝对角度 0 rad，不会重新标定电机零点。

收到零角度命令的合法反馈只表示电机接受了命令，不代表电机已经物理到达零点。当前程序没有等待角度收敛的判断。

---

## 4. 视觉数据链路

### 4.1 视觉帧格式

程序要求的文本格式为：

~~~text
GB,帧序号,有效位,位置\r\n
~~~

示例：

~~~text
GB,38,1,-605\r\n
~~~

解析结果：

~~~text
frame_seq    = 38
valid        = 1
position_raw = -605
position_cm  = -6.05 cm
~~~

位置字段按固定两位小数换算：

~~~c
position_cm = position_raw / 100.0f;
~~~

示例：

| 原始位置 | 解析位置 |
|---:|---:|
| 100 | 1.00 cm |
| -605 | -6.05 cm |
| 1234 | 12.34 cm |

### 4.2 HAL_UART_RxCpltCallback()

USART1 收到一个字节后执行：

~~~c
APP_Control_RxByte(&control, vision_rx_byte);
HAL_UART_Receive_IT(&huart1, &vision_rx_byte, 1U);
~~~

中断回调不做字符串解析、浮点运算或 PID，只完成字节入队和重新接收，从而缩短中断执行时间。

USART6 收满 10 字节反馈后执行：

~~~c
BSP_MOTOR_OnRxComplete(&motor);
~~~

### 4.3 APP_Control_RxByte()

函数原型：

~~~c
void APP_Control_RxByte(app_control_t *ctl, uint8_t byte);
~~~

作用：把 USART1 收到的一个字节压入 128 字节环形缓冲区。

关键字段：

| 字段 | 作用 |
|---|---|
| rx_head | 中断写入位置 |
| rx_tail | 主循环读取位置 |
| rx_overflow | 缓冲区或帧溢出标志 |
| rx_ring[128] | 原始字节环形缓冲区 |

如果缓冲区已满，当前字节会被丢弃并设置 rx_overflow。程序会等到下一个换行符后再重新接收一帧。

### 4.4 APP_Control_ProcessRx()

函数原型：

~~~c
app_control_err_t APP_Control_ProcessRx(app_control_t *ctl,
                                        uint32_t now_ms);
~~~

主循环持续调用该函数。它负责：

- 从环形缓冲区取字节；
- 忽略回车符；
- 使用换行符判断一帧结束；
- 限制一帧最多 63 个有效字符；
- 调用 APP_Vision_Parse()；
- 解析成功后提交新的位置数据。

解析成功会更新：

~~~text
control.position_cm
control.frame_seq
control.vision_valid
control.frame_seen
control.last_vision_ms
~~~

其中：

| 字段 | 含义 |
|---|---|
| position_cm | PID 使用的小球测量位置 |
| frame_seq | 最新视觉帧序号 |
| vision_valid | 视觉模块给出的有效位 |
| frame_seen | 是否收到过一帧合法数据 |
| last_vision_ms | 该帧被主循环解析时的 HAL tick |

frame_seq 当前只保存，没有参与去重、丢帧判断或乱序判断。

### 4.5 APP_Vision_Parse()

函数原型：

~~~c
app_vision_err_t APP_Vision_Parse(const uint8_t *frame,
                                  uint16_t len,
                                  app_vision_data_t *data);
~~~

作用：解析一帧已经结束并以空字符结尾的视觉文本。

返回值：

| 返回值 | 含义 |
|---|---|
| APP_VISION_OK | 解析成功 |
| APP_VISION_ERR_ARG | 空指针或长度不足 |
| APP_VISION_ERR_FORMAT | 前缀、逗号、有效位或结尾格式错误 |
| APP_VISION_ERR_RANGE | 数字超出支持范围 |

解析失败不会覆盖上一帧已保存的位置。

---

## 5. 控制状态机

### 5.1 应用状态

| 状态 | 含义 |
|---|---|
| APP_CONTROL_DISABLED | 电机未使能 |
| APP_CONTROL_STARTUP | 上电使能和零角度命令正在执行 |
| APP_CONTROL_READY | 电机已使能，但没有可用于平衡的新鲜视觉数据 |
| APP_CONTROL_BALANCE | 正在根据视觉数据执行 PID |
| APP_CONTROL_FAULT | PID、通信或电机状态出现故障 |

状态转换：

~~~text
初始化
→ DISABLED

APP_Control_Start()
→ STARTUP

使能反馈成功
→ 发送绝对角度 0 rad
→ 零角度命令反馈成功
→ READY

收到合法、valid=1、未超时的视觉数据
→ BALANCE

无视觉帧 / valid=0 / 超过100 ms
→ READY，并清空 PID 历史

按键失能完成
→ DISABLED

PID错误 / CRC错误 / ID错误 / DMA错误 / 通信超时
→ FAULT，并尝试失能电机
~~~

FAULT 状态目前没有专门的恢复 API。APP_Control_SetEnabled(..., 1) 不允许直接从 FAULT 重新使能。

### 5.2 电机操作状态

motor_op 表示应用层当前等待完成的电机事务：

| 状态 | 含义 |
|---|---|
| APP_CONTROL_MOTOR_NONE | 当前没有电机事务 |
| APP_CONTROL_MOTOR_ENABLE | 等待使能反馈 |
| APP_CONTROL_MOTOR_DISABLE | 等待失能反馈 |
| APP_CONTROL_MOTOR_ANGLE | 等待角度命令反馈 |

该状态与 APP_CONTROL_BALANCE 等应用状态不是同一层含义。

---

## 6. APP_Control_Tick() 核心调度

函数原型：

~~~c
app_control_err_t APP_Control_Tick(app_control_t *ctl,
                                   uint32_t now_ms);
~~~

主循环会高频调用该函数，但函数不会在每次调用时都计算 PID。

### 6.1 推进上一条电机事务

首先调用：

~~~text
APP_Control_ProcessMotor()
→ BSP_MOTOR_Process()
~~~

如果上一条电机命令的 TX 或 RX DMA 还没有完成：

~~~text
busy = 1
APP_Control_Tick() 返回 APP_CONTROL_OK
~~~

因此同一时刻只允许一个电机事务。

### 6.2 检查运行条件

以下情况不会计算 PID：

- 当前电机事务尚未完成；
- 状态为 APP_CONTROL_FAULT；
- 电机没有使能；
- 从未收到合法视觉帧；
- 视觉数据超过 100 ms；
- 视觉帧的 valid 为 0。

视觉无效或超时时会：

~~~text
state = APP_CONTROL_READY
APP_PID_Reset()
frame_seen = 0
vision_valid = 0
last_ctrl_ms = 0
~~~

之后必须收到一帧新的合法视觉数据才能重新进入平衡。

### 6.3 检查控制周期

第一次获得有效视觉数据时只记录：

~~~c
last_ctrl_ms = now_ms;
~~~

不会立刻计算 PID。

后续满足以下条件才进行一次控制：

~~~c
now_ms - last_ctrl_ms >= 17U
~~~

17 ms 对应：

~~~text
1000 / 17 ≈ 58.82 Hz
~~~

所以当前只是接近 60 Hz，并不是硬件定时器产生的严格 60 Hz。

PID 使用实际经过时间：

~~~c
dt = elapsed_ms / 1000.0f;
~~~

如果一次电机通信导致间隔变成 25 ms，则该次 PID 使用 dt=0.025 s。

### 6.4 执行 PID

~~~c
APP_PID_Update(ctl->pid,
               ctl->target_cm,
               ctl->position_cm,
               dt,
               &angle);
~~~

输入：

| 参数 | 含义 | 单位 |
|---|---|---|
| target_cm | 小球目标位置 | cm |
| position_cm | 视觉测量位置 | cm |
| dt | 实际控制间隔 | s |

输出 angle 是相对电机零点的目标角度，单位 rad。

### 6.5 电机方向和机械限位

~~~c
angle *= ctl->motor_dir;
angle = Clamp(angle, -ctl->angle_max, ctl->angle_max);
~~~

motor_dir 只能是 +1.0 或 -1.0：

- +1.0：保持 PID 输出方向；
- -1.0：反转 P、I、D 合成后的整体方向。

真实机械方向必须根据实机确认，不能只根据源码推断。

当前 angle_max 为 0.45 rad，约为 25.8°。

PID 内部已有一次 -0.45～+0.45 rad 输出限幅，APP_Control_Tick() 在发送前又做一次机械限幅，形成两层软件保护。

### 6.6 有符号相对角度映射到绝对角度

应用控制角度范围：

~~~text
-0.45～+0.45 rad
~~~

电机绝对角度 API 接受：

~~~text
0～2π rad
~~~

映射规则：

~~~c
if (angle < 0.0f)
{
    motor_angle = 2.0f * pi + angle;
}
else
{
    motor_angle = angle;
}
~~~

示例：

| 应用层相对角度 | 发送给电机的绝对角度 |
|---:|---:|
| +0.20 rad | 0.20 rad |
| 0 rad | 0 rad |
| -0.20 rad | 2π-0.20 ≈ 6.083 rad |
| -0.45 rad | 2π-0.45 ≈ 5.833 rad |

这仍是 0x05 绝对角度模式，不是 0x07 角度步进模式。

### 6.7 启动角度事务

~~~c
BSP_MOTOR_SetAngle(ctl->motor, angle);
ctl->motor_op = APP_CONTROL_MOTOR_ANGLE;
~~~

BSP_MOTOR_SetAngle() 返回 BSP_MOTOR_OK 只表示 DMA 事务成功启动，不表示：

- 命令已经完整发送；
- 电机反馈已经返回；
- 电机已经运动到目标位置。

事务完成要等后续 APP_Control_Tick() 调用 BSP_MOTOR_Process() 才能确认。

---

## 7. PID 数学实现

PID 实现位于 APP/src/app_pid.c。

### 7.1 误差

~~~c
error = target - measure;
~~~

即：

~~~text
e = x_target - x_ball
~~~

当前目标位置为 0 cm。

例如：

~~~text
target   = 0 cm
position = +5 cm
error    = -5 cm
~~~

如果 Kp 大于 0，则 P 输出为负。最终机械作用方向还会受到 motor_dir 影响。

### 7.2 比例项

~~~text
uP = Kp × error
~~~

Kp 的单位：

~~~text
rad/cm
~~~

比例项决定当前位置误差产生多大的杆倾角。

### 7.3 积分项

~~~c
integral = integral + error * dt;
~~~

积分状态限制为：

~~~text
-1 ≤ integral ≤ +1
~~~

积分输出：

~~~text
uI = Ki × integral
~~~

单位：

| 量 | 单位 |
|---|---|
| integral | cm·s |
| Ki | rad/(cm·s) |
| uI | rad |

积分状态最大为 ±1，因此积分项最大角度贡献是 ±Ki rad，而不是固定的 ±1 rad。

### 7.4 微分项

代码对测量值做微分：

~~~c
derivative = (measure - prev_measure) / dt;
~~~

微分输出：

~~~text
uD = -Kd × derivative
~~~

即：

~~~text
uD = -Kd × d(position)/dt
~~~

Kd 的单位：

~~~text
rad·s/cm
~~~

如果小球以 +10 cm/s 向正方向运动：

~~~text
uD = -10 × Kd rad
~~~

它会倾向于阻止小球继续向正方向运动，因此主要提供阻尼。

使用测量微分可以避免目标位置突然改变时产生明显的微分冲击。

### 7.5 完整公式

~~~text
u = Kp × e
  + Ki × integral(e dt)
  - Kd × d(position)/dt
~~~

然后执行：

~~~text
u_limited = clamp(u, -0.45, +0.45)
angle_cmd = clamp(u_limited × motor_dir, -0.45, +0.45)
~~~

### 7.6 抗积分饱和

当原始输出已经超过上限，并且误差仍推动输出向同一饱和方向增加时，代码暂停本次积分累积。

例如：

~~~text
raw_output > out_max
error > 0
~~~

此时保持旧的 integral，防止积分继续增大。

这可以减轻小球长时间偏在一侧后，回到目标点时产生严重过冲。

### 7.7 第一次微分

APP_PID_Reset() 后 initialized 为 0。

第一次 PID 更新时：

~~~text
derivative = 0
~~~

第二次更新才使用前后两个位置计算速度，避免第一次计算使用无效的历史测量。

---

## 8. PID API

### 8.1 APP_PID_Init()

~~~c
APP_PID_Init(pid, kp, ki, kd,
             out_min, out_max,
             i_min, i_max);
~~~

作用：

- 设置 Kp、Ki、Kd；
- 设置输出限幅；
- 设置积分状态限幅；
- 清空积分和微分历史。

### 8.2 APP_PID_SetGains()

~~~c
APP_PID_SetGains(pid, kp, ki, kd);
~~~

作用：运行时修改 PID 参数。

该函数不会自动清空 integral、prev_measure 和 initialized。如果需要从干净状态使用新参数，应再调用 APP_PID_Reset()。

当前主程序没有调用该 API，因此目前参数是在 main.c 中通过宏设置，修改后需要重新构建和下载。

### 8.3 APP_PID_SetOutputLimit()

~~~c
APP_PID_SetOutputLimit(pid, out_min, out_max);
~~~

作用：修改 PID 最终输出范围。

当前范围是 -0.45～+0.45 rad。

### 8.4 APP_PID_SetIntegralLimit()

~~~c
APP_PID_SetIntegralLimit(pid, i_min, i_max);
~~~

作用：

- 修改积分状态范围；
- 立即把当前积分值限制到新范围。

要求 i_min 不大于 0，i_max 不小于 0。

### 8.5 APP_PID_Reset()

~~~c
APP_PID_Reset(pid);
~~~

清除：

~~~text
integral     = 0
prev_measure = 0
initialized  = 0
~~~

控制代码在以下情况自动调用：

- 控制器初始化；
- 电机失能；
- 视觉无效；
- 视觉超时；
- 进入故障；
- 上电归零完成。

### 8.6 APP_PID_Update()

~~~c
APP_PID_Update(pid, target, measure, dt, &output);
~~~

作用：执行一次 PID 运算。

要求：

- target 和 measure 必须是有限浮点数；
- dt 必须大于 0；
- output 指针不能为空。

返回值：

| 返回值 | 含义 |
|---|---|
| APP_PID_OK | 运算成功 |
| APP_PID_ERR_ARG | 指针参数错误 |
| APP_PID_ERR_RANGE | 限幅参数错误 |
| APP_PID_ERR_DT | dt 非法 |
| APP_PID_ERR_NUM | 输入或计算结果不是有效有限数 |

---

## 9. 电机 BSP API

### 9.1 BSP_MOTOR_Init()

~~~c
BSP_MOTOR_Init(dev, uart, id, timeout);
~~~

初始化电机通信上下文并验证 DMA 配置，不发送任何命令。

### 9.2 BSP_MOTOR_Enable()

~~~c
BSP_MOTOR_Enable(dev);
~~~

异步启动电机使能事务：

~~~text
命令码 = 0x01
控制值 = 0
~~~

### 9.3 BSP_MOTOR_Disable()

~~~c
BSP_MOTOR_Disable(dev);
~~~

异步启动电机失能事务：

~~~text
命令码 = 0x02
控制值 = 0
~~~

### 9.4 BSP_MOTOR_SetAngle()

~~~c
BSP_MOTOR_SetAngle(dev, angle);
~~~

发送绝对角度命令：

~~~text
命令码 = 0x05
输入范围 = 0～2π rad
~~~

浮点角度编码为 uint16：

~~~text
raw = round(angle / 2π × 65535)
~~~

控制值按照小端顺序写入发送帧。

### 9.5 BSP_MOTOR_Process()

~~~c
BSP_MOTOR_Process(dev, now_ms, &feedback);
~~~

主循环必须持续调用该 API 推进当前事务。

返回值：

| 返回值 | 含义 |
|---|---|
| BSP_MOTOR_BUSY | TX/RX 尚未全部完成 |
| BSP_MOTOR_OK | 反馈完整且 ID、CRC 正确 |
| BSP_MOTOR_ERR_ARG | 参数或上下文无效 |
| BSP_MOTOR_ERR_RANGE | 角度超出范围 |
| BSP_MOTOR_ERR_STATE | 当前没有活动事务或状态错误 |
| BSP_MOTOR_ERR_DMA | DMA/UART 报错 |
| BSP_MOTOR_ERR_TX | TX DMA 启动失败 |
| BSP_MOTOR_ERR_RX | RX DMA 启动失败 |
| BSP_MOTOR_ERR_TIMEOUT | 事务超过设定超时 |
| BSP_MOTOR_ERR_ID | 反馈电机 ID 不匹配 |
| BSP_MOTOR_ERR_CRC | 反馈 CRC 错误 |

成功后解析：

~~~text
feedback.state
feedback.enabled
feedback.current
feedback.speed
feedback.angle
~~~

当前这些反馈不进入球位置 PID，只用于通信确认、使能检查和调试观察。

### 9.6 BSP_MOTOR_Cancel()

~~~c
BSP_MOTOR_Cancel(dev);
~~~

停止当前 UART DMA 事务并清除：

~~~text
active
tx_done
rx_done
dma_error
~~~

进入控制故障时会调用。

### 9.7 BSP_MOTOR_OnTxComplete()

~~~c
BSP_MOTOR_OnTxComplete(dev);
~~~

由 HAL_UART_TxCpltCallback() 调用，将 tx_done 置 1。

不要在普通应用代码中主动模拟调用。

### 9.8 BSP_MOTOR_OnRxComplete()

~~~c
BSP_MOTOR_OnRxComplete(dev);
~~~

由 HAL_UART_RxCpltCallback() 调用，将 rx_done 置 1。

### 9.9 BSP_MOTOR_OnError()

~~~c
BSP_MOTOR_OnError(dev);
~~~

由 HAL_UART_ErrorCallback() 调用，将 dma_error 置 1。

下一次 BSP_MOTOR_Process() 会识别该错误、停止 DMA 并返回 BSP_MOTOR_ERR_DMA。

---

## 10. 电机协议的一发一收

每条命令发送 5 字节：

| 字节 | 含义 |
|---:|---|
| tx[0] | 电机 ID |
| tx[1] | 命令码 |
| tx[2] | 控制值低字节 |
| tx[3] | 控制值高字节 |
| tx[4] | CRC8 |

CRC 参数：

~~~text
初值       = 0x00
多项式     = 0x07
计算范围   = tx[0]～tx[3]
~~~

每条命令的启动顺序：

~~~c
HAL_UART_Receive_DMA(uart, rx, 10);
HAL_UART_Transmit_DMA(uart, tx, 5);
~~~

先挂接 RX DMA，再启动 TX DMA，目的是避免电机快速回复时丢失反馈开头的字节。

反馈共 10 字节：

| 字节 | 当前代码解释 |
|---:|---|
| rx[0] | 电机 ID |
| rx[1] | 状态，最低位作为 enabled |
| rx[2] | 当前代码未使用 |
| rx[3..4] | 电流 |
| rx[5..6] | 速度 |
| rx[7..8] | 绝对角度 |
| rx[9] | CRC8 |

电机反馈角度范围是 0～2π。调试时转换成相对零点的有符号角度：

~~~text
反馈角度 ≤ π：
relative_angle = feedback.angle

反馈角度 > π：
relative_angle = feedback.angle - 2π
~~~

例如：

~~~text
feedback.angle = 6.083 rad
relative_angle = 6.083 - 6.283
               = -0.200 rad
~~~

---

## 11. 应用控制层 API

### 11.1 APP_Control_Init()

~~~c
APP_Control_Init(ctl,
                 motor,
                 pid,
                 target_cm,
                 motor_dir,
                 angle_max,
                 ctrl_period_ms,
                 vision_timeout_ms);
~~~

作用：

- 关联电机上下文和 PID；
- 设置目标位置；
- 设置机械方向；
- 设置角度范围；
- 设置控制周期和视觉超时；
- 初始化控制、视觉和通信状态；
- 复位 PID。

### 11.2 APP_Control_Start()

~~~c
APP_Control_Start(ctl);
~~~

启动上电序列：

~~~text
使能
→ 等待使能反馈
→ 发送绝对角度 0
→ 等待角度反馈
→ READY
~~~

它与普通重新使能不同，只在上电初始化流程中调用。

### 11.3 APP_Control_RxByte()

~~~c
APP_Control_RxByte(ctl, byte);
~~~

在 USART1 接收中断回调中调用，只负责把一个字节写入环形缓冲区。

### 11.4 APP_Control_ProcessRx()

~~~c
APP_Control_ProcessRx(ctl, now_ms);
~~~

在主循环中调用，负责拼接和解析视觉帧。

### 11.5 APP_Control_SetEnabled()

~~~c
APP_Control_SetEnabled(ctl, 1U);  // 使能
APP_Control_SetEnabled(ctl, 0U);  // 失能
~~~

当前 PA0 按键调用该 API。

需要注意：

- API 只异步启动命令；
- 已有活动事务时返回 APP_CONTROL_BUSY；
- 从普通 DISABLED 状态重新使能不会再发送零角度命令；
- FAULT 状态不能直接重新使能。

### 11.6 APP_Control_Tick()

~~~c
APP_Control_Tick(ctl, now_ms);
~~~

整个控制系统的调度中心：

~~~text
处理电机反馈
→ 完成启动、使能或失能状态转换
→ 检查视觉有效性
→ 检查控制周期
→ PID
→ 电机方向修正
→ 角度限位
→ 绝对角度映射
→ 启动电机 DMA 事务
~~~

返回值：

| 返回值 | 含义 |
|---|---|
| APP_CONTROL_OK | 本次调度正常，包括无需控制或事务仍忙 |
| APP_CONTROL_BUSY | 请求启动操作时已有事务 |
| APP_CONTROL_ERR_ARG | 上下文或指针参数错误 |
| APP_CONTROL_ERR_RANGE | 初始化参数超出范围 |
| APP_CONTROL_ERR_STATE | 当前状态不允许该操作 |
| APP_CONTROL_ERR_PID | PID 运算错误 |
| APP_CONTROL_ERR_MOTOR | 电机事务或反馈错误 |

---

## 12. 当前可调参数

参数位于 Src/main.c。

| 参数 | 当前值 | 作用 |
|---|---:|---|
| MOTOR_ID | 0x01 | 电机协议地址 |
| MOTOR_TIMEOUT_MS | 100 ms | 单次电机事务超时 |
| VISION_TIMEOUT_MS | 100 ms | 视觉数据失效阈值 |
| CONTROL_PERIOD_MS | 17 ms | PID 标称周期 |
| BALANCE_TARGET_CM | 0.0 cm | 小球目标位置 |
| BALANCE_MOTOR_DIR | +1.0 | 最终角度方向 |
| BALANCE_ANGLE_LIMIT_RAD | 0.45 rad | 最大正负目标角度 |
| BALANCE_PID_KP | 0.0 | 比例系数 |
| BALANCE_PID_KI | 0.0 | 积分系数 |
| BALANCE_PID_KD | 0.0 | 微分系数 |
| BALANCE_PID_I_LIMIT | 1.0 | 积分状态限幅 |

参数物理单位：

| 参数或变量 | 单位 |
|---|---|
| Kp | rad/cm |
| Ki | rad/(cm·s) |
| Kd | rad·s/cm |
| PID 输出 | rad |
| integral | cm·s |
| 视觉位置 | cm |
| dt | s |

例如只使用比例项时：

~~~text
误差 = 5 cm
Kp   = 0.02 rad/cm
P输出 = 0.10 rad
~~~

该数值只用于说明单位关系，不是实机推荐参数。

---

## 13. 建议调参顺序

### 13.1 确认视觉坐标

首先观察 control.position_cm：

- 球位于目标点时是否接近 0 cm；
- 球向两侧移动时数值正负是否稳定；
- 是否有明显跳变和噪声；
- vision_valid 是否稳定为 1；
- frame_seq 是否连续变化。

如果物理中心不是视觉 0 cm，应先校准视觉坐标或修改 BALANCE_TARGET_CM，不要依赖积分项强行补偿错误的目标坐标。

### 13.2 确认控制方向

先保持：

~~~text
Ki = 0
Kd = 0
~~~

只设置一个很小的非零 Kp。

当前误差公式：

~~~text
error = target - position
~~~

当目标为 0、球位置为正时：

~~~text
error < 0
PID 输出 < 0
~~~

如果该角度能让球向中心加速，则 motor_dir 正确；如果让球继续远离中心，则必须反转 BALANCE_MOTOR_DIR。

控制方向没有确认前不能继续增大 Kp，否则系统会形成正反馈。

### 13.3 只调 Kp

保持：

~~~text
Ki = 0
Kd = 0
~~~

逐步增加 Kp：

- Kp 太小：杆倾角小，球回中心很慢；
- Kp 增大：回中心速度提高；
- Kp 太大：球持续来回摆动，角度可能频繁达到限幅。

找到可以明显回正、但开始出现持续振荡的范围后，不再继续增大。

### 13.4 增加 Kd

Kd 根据小球速度提供阻尼：

- Kd 太小：球经过中心后仍高速冲向另一侧；
- 合适的 Kd：接近中心时提前减小倾角，减少过冲；
- Kd 太大：系统反应迟钝，视觉噪声还会造成角度指令抖动。

微分会放大视觉位置跳变，因此应同步确认视觉测量质量。

### 13.5 最后增加 Ki

只有 PD 已经基本稳定但仍存在长期静差时，才增加较小的 Ki。

可能需要 Ki 的情况：

- 杆的机械零点有轻微偏差；
- 两侧摩擦不对称；
- 球长期稳定在离目标点一小段距离的位置。

Ki 太大会产生：

- 慢周期摆动；
- 长时间偏离后积分积累过多；
- 小球重新出现时输出突然变大。

---

## 14. 调试器建议观察变量

| 变量 | 含义 |
|---|---|
| control.state | 当前控制状态 |
| control.position_cm | 小球视觉位置 |
| control.target_cm | 小球目标位置 |
| control.vision_valid | 当前视觉有效位 |
| control.frame_seen | 是否收到合法帧 |
| control.frame_seq | 最新视觉帧序号 |
| control.last_vision_ms | 最新视觉帧解析时间 |
| control.last_ctrl_ms | 上一次 PID 时间 |
| control.motor_enabled | 软件确认的电机使能状态 |
| control.motor_op | 当前等待完成的电机事务 |
| control.motor_err | 最近一次电机错误 |
| control.motor_fb.angle | 电机反馈绝对角度 |
| control.motor_fb.speed | 电机反馈速度 |
| control.motor_fb.current | 电机反馈电流 |
| balance_pid.integral | PID 积分状态 |
| balance_pid.prev_measure | 微分使用的上一位置 |
| balance_pid.initialized | 微分历史是否有效 |
| motor.active | 是否有活动 DMA 事务 |
| motor.tx_done | TX DMA 是否完成 |
| motor.rx_done | RX DMA 是否完成 |
| motor.dma_error | UART/DMA 是否出错 |

当前代码没有单独保存 P、I、D 三项和最终发送的有符号角度，因此调参时只能通过当前参数和上下文推算这些量。

---

## 15. 调参前必须了解的现有行为

### 15.1 当前 PID 输出恒为零

当前：

~~~text
Kp = 0
Ki = 0
Kd = 0
~~~

所以视觉控制不会产生非零角度。

### 15.2 视觉失效时不会主动回零

视觉超时或 valid=0 时，状态回到 READY 并复位 PID，但程序不会发送 0 rad。

由于使用绝对角度模式，电机可能继续保持最后一个目标角度。因此视觉丢失时杆可能保持倾斜，而不是自动水平。

### 15.3 同一视觉帧可能被多次使用

frame_seq 目前没有参与控制。只要上一帧没有超过 100 ms，同一个 position_cm 可能被多个 PID 周期重复使用。

### 15.4 控制周期不是严格 60 Hz

17 ms 对应约 58.82 Hz，并由主循环轮询调度。

如果电机一发一收事务超过 17 ms，实际控制周期会进一步变长。PID 会使用实际 dt，但控制频率仍会降低并产生抖动。

### 15.5 电机反馈没有进入 PID

motor_fb.angle、speed 和 current 只用于通信确认和调试。

当前 MCU 闭环反馈只有视觉位置。电机角度控制由电机驱动器内部完成。

### 15.6 ±0.45 rad 是目标角度软件限位

0.45 rad 约为 25.8°。

该限位只限制 MCU 发送的目标角度：

- 不是电机固件硬限位；
- 不是机械硬限位；
- 当前没有根据反馈角度检查实际电机是否越界。

### 15.7 上电零角度命令不等待物理到位

程序收到零角度命令的合法反馈后就进入 READY。如果此时出现有效视觉帧，控制器可能在电机尚未完全到达零点时开始平衡。

### 15.8 主循环忽略了部分返回值

当前主循环对以下调用使用了 void 转换：

~~~c
APP_Control_SetEnabled();
APP_Control_ProcessRx();
APP_Control_Tick();
~~~

因此发生故障时没有日志输出。需要观察 control.state 和 control.motor_err 才能确认错误。

### 15.9 20 ms 按键消抖会短暂阻塞

PA0 第一次检测到按下时执行 HAL_Delay(20U)。

该阻塞只发生在按键触发时，但按下按键的瞬间会暂停主循环、视觉解析和电机事务推进约 20 ms。

---

## 16. 核心理解总结

当前系统调节的是：

> 小球位置误差应该产生多大的电机绝对倾角。

各参数的核心作用：

- Kp：根据小球离目标点的距离产生回正倾角。
- Kd：根据小球运动速度提前制动，减少过冲。
- Ki：消除长期稳定偏差，应最后且小量加入。
- motor_dir：决定整个闭环是负反馈还是危险的正反馈。
- angle_max：限制 MCU 能发送的最大目标倾角。
- vision_timeout_ms：决定视觉数据多久不更新后停止继续计算。
- ctrl_period_ms：决定外环 PID 的标称更新频率。

调参前最优先确认的是：

1. 视觉位置的零点、单位和正负方向正确。
2. 电机零点与机械水平位置一致。
3. motor_dir 形成负反馈。
4. 视觉数据连续、有效且噪声可接受。
5. 电机通信事务能够稳定在控制周期内完成。

