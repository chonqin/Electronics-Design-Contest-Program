# 操作记录

## 2026-06-11 创建BSP电机与编码器驱动

### 操作内容
在BSP目录下创建了电机驱动和编码器驱动文件：

**新增文件：**
- `BSP/inc/bsp_motor.h` — 电机驱动头文件
- `BSP/src/bsp_motor.c` — 电机驱动实现
- `BSP/inc/bsp_encoder.h` — 编码器驱动头文件
- `BSP/src/bsp_encoder.c` — 编码器驱动实现

### 设计说明

**电机驱动 (bsp_motor)：**
- 驱动芯片：TB6612FNG
- 双路电机控制：MOTOR_A (PWMA/AIN1/AIN2)，MOTOR_B (PWMB/BIN1/BIN2)
- PWM周期：4000（80MHz时钟，对应20kHz PWM频率）
- 接口：SetSpeed(正负值控制方向和速度)、Stop(滑行停止)、Brake(刹车)

**编码器驱动 (bsp_encoder)：**
- 电机：MG310霍尔编码电机，减速比1:20，11线编码器
- 计数方式：单相计数（A相上升沿中断 + 读B相判断方向）
- 定时器TIMA0每10ms中断一次，用于采样编码器脉冲数
- 提供 `Encoder_GPIO_IRQHandler()` 供GPIO中断调用
- 提供 `Encoder_Update()` 供定时器中断调用

### 已解决问题
- E2B (PB7) sysconfig配置已由用户修正为数字输入

---

## 2026-06-11 添加代码注释

### 操作内容
为BSP四个文件添加Doxygen风格注释：
- 头文件：`@file`、`@brief`、`@param`、`@return` 描述接口
- 源文件：文件顶部说明工作原理，关键逻辑处加行内注释
- 宏定义附加简短说明

---

## 2026-06-11 创建BSP测试例程

### 操作内容
创建电机+编码器综合测试模块，并更新main.c调用测试。

**新增文件：**
- `BSP/inc/bsp_test.h` — 测试接口声明
- `BSP/src/bsp_test.c` — 测试实现

**修改文件：**
- `main.c` — 调用 `BSP_Test_Motor()`，添加TIMA0和GROUP1中断处理函数

### 测试逻辑
1. 初始化电机和编码器，LED闪烁1次表示启动
2. 电机A以50%占空比正转2s → 读编码器1脉冲验证正方向
3. 电机A反转2s → 读编码器1脉冲验证反方向
4. 电机B重复上述流程
5. 结果指示：
   - LED1亮 = 电机A通过
   - LED2亮 = 电机B通过
   - 双LED闪烁 = 全部通过
   - 对应LED灭 = 该路失败

---

## 2026-06-11 移植OLED驱动（SPI→I2C，STM32→MSPM0）

### 操作内容
将用户提供的STM32 SSD1306 SPI驱动移植为MSPM0G3507硬件I2C驱动。

**修改文件：**
- `BSP/inc/oled.h` — 重写，去掉STM32依赖，支持I2C/SPI宏切换
- `BSP/src/oled.c` — 重写底层通信，使用MSPM0 DL_I2C API

**保留文件：**
- `BSP/inc/oledfont.h` — 字模数据无需修改

### 移植要点
- 通信方式：硬件I2C（sysconfig已配置的I2C1，SDA=PA30，SCL=PA29）
- I2C地址：0x3C
- 无RST引脚，初始化用delay等待上电稳定
- 刷屏优化：水平寻址模式 + 批量写入1024字节
- 通过 `OLED_USE_I2C` 宏（默认=1）切换I2C/SPI模式
- SPI模式保留框架，需额外配置sysconfig后使用

---

## 2026-06-11 添加OLED测试例程

### 操作内容
在bsp_test中新增 `BSP_Test_OLED()` 函数。

**修改文件：**
- `BSP/inc/bsp_test.h` — 添加 `BSP_Test_OLED()` 声明
- `BSP/src/bsp_test.c` — 添加OLED测试实现，include oled.h

### 测试逻辑
1. 初始化OLED，显示文字字符串（验证I2C通信和字符渲染）
2. 画边框+圆形（验证图形绘制）
3. 循环刷新计数器（验证持续通信稳定性）

---

## 2026-06-26 移植ICM42688陀螺仪驱动（STM32→MSPM0）

### 操作模型
Claude Opus 4.8

### 操作内容
将STM32平台的ICM42688六轴惯性传感器驱动完整移植到MSPM0G3507平台。

**新增文件：**
- `BSP/inc/bsp_icm42688.h` — ICM42688驱动头文件
- `BSP/src/bsp_icm42688.c` — ICM42688驱动实现(SPI通信)
- `APP/imu.h` — IMU姿态解算头文件
- `APP/imu.c` — IMU姿态解算实现(AHRS四元数算法)
- `test_icm42688.c` — 测试示例代码
- `ICM42688移植说明.md` — 完整移植文档

### 移植要点

**硬件配置：**
- SPI通信：SPI_IMU (8MHz, Mode 3, 8bit, MSB先行)
- 引脚：SCLK(PA12), MOSI(PB19), MISO(PA14), CS(PB25)
- SPI模式：CPOL=1, CPHA=1 (对应STM32的Mode 3)

**底层适配：**
1. SPI通信层
   - STM32 HAL: `HAL_SPI_TransmitReceive()` 
   - TI DriverLib: `DL_SPI_transmitDataBlocking8()` + `DL_SPI_receiveDataBlocking8()`

2. GPIO控制
   - STM32 HAL: `HAL_GPIO_WritePin()`
   - TI DriverLib: `DL_GPIO_setPins()` / `DL_GPIO_clearPins()`

3. 延时函数
   - STM32 HAL: `HAL_Delay(ms)`
   - TI: `delay_cycles((ms) * (CPUCLK_FREQ / 1000))`

**功能实现：**
- 寄存器读写(单字节/连续多字节)
- 设备ID读取验证
- 加速度计配置(默认±4g, 100Hz)
- 陀螺仪配置(默认±1000dps, 1000Hz)
- 温度读取
- 六轴数据读取(加速度+陀螺仪)
- 灵敏度系数计算

**姿态解算：**
- 算法：Mahony六轴互补滤波(AHRS)
- 陀螺仪零偏自动校准(静止时方差检测)
- 加速度合理性检测(0.5g~2g范围)
- 四元数更新(微分方程+归一化)
- 欧拉角转换(Yaw/Pitch/Roll)
- 需要5ms定时器支持(200Hz采样)

### 配置参数
- 加速度量程：±4g
- 陀螺仪量程：±1000dps
- 加速度采样率：100Hz
- 陀螺仪采样率：1000Hz
- 工作模式：低噪声模式
- 滤波器参数：Kp=0.6, Ki=0.001

### 使用说明
1. **基础测试**：调用`ICM_Init()`初始化，`bsp_IcmGetRawData()`读取数据
2. **姿态解算**：需配置5ms定时器，在中断中调用`IMU_sample()`和递增`nowtime`变量
3. **零偏校准**：首次使用需静止10秒进行陀螺仪零偏校准
4. **主循环调用**：`IMU_getYawPitchRoll(angles)`获取姿态角

### 测试建议
1. 先测试设备ID读取(`ICM_ReadID()`)验证SPI通信
2. 测试原始数据读取，检查数值是否合理
3. 配置定时器后测试姿态解算
4. 静止时观察姿态角是否稳定，晃动时是否响应

### 注意事项
- 上电后需等待100ms再配置寄存器
- 写入PWR_MGMT0后200us内禁止读写操作
- 姿态解算依赖定时器中断，采样频率固定5ms
- 零偏校准期间必须保持传感器静止

---

## 2026-07-12 修复OLED I2C通信bug

### 操作模型
Claude Opus 4.8

### 操作内容
修复 `BSP/src/oled.c` 中 I2C 底层通信的两处bug。

**修改文件：**
- `BSP/src/oled.c`

### Bug详情

**Bug1：传输完成等待标志错误（`OLED_I2C_Write`）**
- 旧：`while (... DL_I2C_CONTROLLER_STATUS_BUSY_BUS)` — BUSY_BUS 不代表传输结束
- 新：`while (!(... DL_I2C_CONTROLLER_STATUS_IDLE))` — 等控制器真正进入空闲

**Bug2：`OLED_I2C_WriteBuf` FIFO 等待条件错误导致下溢**
- 旧：`while (!DL_I2C_isControllerTXFIFOEmpty(...))` — 等FIFO空再填，FIFO空时I2C控制器无数据可发，引发总线下溢
- 新：`while (DL_I2C_isControllerTXFIFOFull(...))` — 等FIFO有空位即填入，保持数据连续

### 操作模型
Claude Opus 4.8

### 操作内容
阅读全部源码文件，梳理工程结构与各模块职责。

### 工程现状
- 主入口 `main.c` 当前运行 OLED 测试（`BSP_Test_OLED()`），电机/按键测试已注释
- BSP 层完整：电机(TB6612)、编码器(MG310)、ICM42688(SPI)、OLED(I2C)、按键 均已实现
- APP 层已有 IMU Mahony 姿态解算，但尚未集成进主循环（缺少5ms定时器中断驱动）

---

## 2026-07-16 修复 ICM42688P IMU yaw 动态漂移问题

### 操作模型
GPT-5

### 操作时间
2026-07-16 19:21:35 +08:00

### 用户确认
用户反馈上电后旋转五圈回到同一起点时 yaw 漂移约 20 度，并在修改后确认可以记录日志。

### 操作内容
修复 IMU 姿态解算在动态旋转时丢采样和积分时基不一致导致的 yaw 漂移问题。

**修改文件：**
- `APP/src/imu.c`
- `BSP/src/bsp_icm42688.c`
- `BSP/src/bsp_test.c`

### 关键修改
- 将 Mahony 姿态积分从主循环读取函数移动到 `IMU_sample()`，保证每个 10ms 定时采样点都参与积分，避免 OLED 刷新和串口打印导致中间角速度样本被丢弃。
- `IMU_getYawPitchRoll()` 改为只读取缓存的 yaw/pitch/roll，减少主循环执行时间对姿态解算的影响。
- 修正 `nowtime` 与 SysConfig 中 `TIMER_IMU_TICK = 10 ms` 的时基关系，定时中断每次仅递增 1 tick。
- 陀螺仪零偏改为在静止窗口方差满足阈值后建立，并在初次稳定后重置融合状态。
- 将 ICM42688P 陀螺仪量程调整为 `±1000 dps`，降低快速连续旋转时超量程削顶导致 yaw 回零误差的风险。
- 修复 ICM42688P 连续寄存器读取流程，确保 burst read 前正确发送读寄存器地址。
- 修复 `PWR_MGMT0` 配置，确保温度、陀螺仪、加速度计进入预期工作模式。

### 验证
- 使用 CCS 工具链执行完整构建：
  `D:\TI\ccs\utils\bin\gmake.exe -C Debug all -B`
- 编译和链接通过，生成 `Debug/Board_Test.out`。

---

## 2026-07-22 修复电机 duty 小值满转问题

### 操作模型
GPT-5

### 操作时间
2026-07-22 14:14:43 +08:00

### 用户确认
用户反馈运行 `Test_Motor()` 时 duty 给定任意值电机都满转，修复后确认“现在正常了”。

### 操作内容
修复 TB6612FNG 电机驱动中 PWM duty 转换错误。

**修改文件：**
- `BSP/src/bsp_motor.c`
- `BSP/inc/bsp_motor.h`

### 关键修改
- 修正 `Motor_SetDuty()` 中正负 duty 到 PWM 比较值的转换逻辑，使用 duty 绝对值作为 PWM 比较值。
- 将 PWM 比较值限幅到 `0..MOTOR_PWM_PERIOD`，避免小 duty 被转换成超大 `uint16_t` 后导致满占空比输出。
- 同步修正 `Motor_SetDuty()` 接口注释中的参数名与含义。
