# 硬件信息

## 主控芯片 (MCU)

- 型号：MSPM0G3507

## 时钟
使用最大主频80mhz

### 通信接口与传感器
- LED: PA7、PB2
- UART：UART0_TX对应PA10 UART0_RX对应PA11，UART1_TX对应PA8 UART1_RX对应PA9。UART0 用于发送调试信息，uart1用于与视觉模块通信。
- 开启SPI外设用于陀螺仪：PICO->PA14 , POCI->PB19 , sclk->pa12 cs->pb25,int->pa16陀螺仪型号为icm42688p 。
- 开启i2c外设用于OLED：SDA->PA30、SCL->PA29
- 电机驱动：使用TB6612芯片驱动两个电机PWMA接PA15，PWMB接PA24
编码器：E2A->pb9 E2B->pb8 。E1A->pb7 E1B->pb6
电机1：对应编码器E1。AIN1接PB10，AIN2接PB13。
电机2：对应编码器E2。BIN1接PB15，IN2接PB16。
使用mg310电机与霍尔编码器，1：20减速比，13线数

- 按键：key1、key2、key3分别对应PB18、pa13、pa17、通过并联一个100nf电容后接地。

- 循迹模块：X1-X8：PA22 PB20 PB21 PB22 PB23 PB24 PA25 PB27，小车前进方向来看，从左至右为X1-X8。检测到黑线时 GPIO 为低电平、对应位为0；未检测到黑线时 GPIO 为高电平、对应位为1。

- 任务3计时输入：PB0，配置为上拉输入；高电平开始计时，低电平停止并锁定显示时间。

## IDE / 工具链

- IDE：TI CCS
- SDK / 驱动库：TI官方通过sysconfig 生成的代码

## 备注
此硬件信息对应自己设计的MSPM0G3507电赛开发板，请使用TI的官方配置工具生成底层代码。
