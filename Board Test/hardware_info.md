# 硬件信息

## 主控芯片 (MCU)

- 型号：MSPM0G3507

## 时钟
使用最大主频80mhz

### 通信接口与传感器
- LED: PA7、PB2
- UART：UART0_TX对应PA10 UART0_RX对应PA11，UART1_TX对应PA8 UART1_RX对应PA9。
- 开启SPI外设用于陀螺仪：MISO->PA14 , MOSI->PB19 , sclk->pa12 cs->pb25,int->pa16陀螺仪型号为icm42688p 。
- 开启i2c外设用于OLED：SDA->PA30`、`SCL->PA29
- 电机驱动：电机所需的定时器请自行开启
编码器：E2A->pb9 E2B->pb8 E1A->pb7 E1B->pb6
电机：PWMA->pa15,AIN1->PB10 ,AIN2->PB13,BIN1->pb15,BIN2->PB16,PWMB->PA24
使用mg310电机与霍尔编码器，1：20减速比，13线数

- 按键：key1、key2、key3分别对应PB18、pa13、pa17、通过并联一个100nf电容后接地。

## IDE / 工具链

- IDE：TI CCS
- SDK / 驱动库：TI官方通过sysconfig 生成的代码

## 备注
此硬件信息对应自己设计的MSPM0G3507电赛开发板，请使用TI的官方配置工具生成底层代码。
