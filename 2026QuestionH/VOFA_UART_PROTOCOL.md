# UART0 与 VOFA+ 调试协议

本文档描述当前工程中 UART0 与 VOFA+ 之间的收发格式。

## 1. 串口配置

UART0 用于底盘调试和在线 PID 调参：

| 参数 | 配置 |
|---|---|
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验位 | None |
| 停止位 | 1 |
| 流控 | None |
| UART0 TX | PA10 |
| UART0 RX | PA11 |

UART0 下行遥测使用 VOFA+ 的 `JustFloat` 协议，上行调参使用 ASCII 文本命令。

## 2. 下行遥测格式

下位机每 100 ms 发送一帧 JustFloat 数据。每帧包含：

- 16 个 IEEE-754 `float` 数据，共 64 字节；
- 4 字节 JustFloat 帧尾：`00 00 80 7F`；
- 每帧总长度为 68 字节。

在 VOFA+ 中选择 `JustFloat`，通道顺序如下：

| 通道 | 名称 | 含义 |
|---:|---|---|
| 0 | `track_pos` | 循迹位置误差 |
| 1 | `enc_l` | 左轮编码器反馈 |
| 2 | `enc_r` | 右轮编码器反馈 |
| 3 | `duty_l` | 左轮 PWM duty |
| 4 | `duty_r` | 右轮 PWM duty |
| 5 | `yaw` | 当前航向角 |
| 6 | `track_kp` | 循迹 PID Kp |
| 7 | `track_ki` | 循迹 PID Ki |
| 8 | `track_kd` | 循迹 PID Kd |
| 9 | `yaw_kp` | 航向 PID Kp |
| 10 | `yaw_ki` | 航向 PID Ki |
| 11 | `yaw_kd` | 航向 PID Kd |
| 12 | `track_mask` | X1-X8 循迹位图 |
| 13 | `mode` | 当前底盘模式 |
| 14 | `cmd_status` | 最近一条命令的执行结果 |
| 15 | `task1_time_s` | 从确认 TASK1 到当前或停车时的用时，单位为秒 |

### 2.1 状态值

`track_mask` 的 bit0-bit7 分别对应 X1-X8，位为 1 表示未检测到黑线，位为 0 表示检测到黑线。

`mode` 的取值为：

```text
0 = STOP
1 = TRACK
2 = TURN
3 = HEADING
```

`cmd_status` 的取值为：

```text
 1 = 命令执行成功
 0 = 尚未收到有效命令或状态未更新
-1 = 命令格式错误或参数名称错误
-2 = 参数不是有限非负数，或计算结果超出 0~100
-3 = 命令超过接收缓冲区长度
```

`task1_time_s` 在任务 1 运行时递增；八路循迹探头全部检测到黑线并停车后，最终帧保留停车用时。

## 3. 上行命令格式

上位机发送 ASCII 文本，每条命令以换行结束。推荐发送 `CRLF`：

```text
命令内容\r\n
```

接收代码会忽略 `\r`，以 `\n` 作为命令结束标志。命令必须使用大写字母，命令中不需要空格。

### 3.1 PID 参数名称

```text
T = Track，循迹 PID
Y = Yaw，航向 PID
P = Kp
I = Ki
D = Kd
```

因此，六个可调参数分别为：

```text
TKP  TKI  TKD
YKP  YKI  YKD
```

### 3.2 设置绝对值

使用 `=` 将参数直接设置为指定值：

```text
TKP=0.18\r\n
TKI=0.00\r\n
TKD=0.00\r\n
YKP=5.40\r\n
YKI=0.45\r\n
YKD=4.50\r\n
```

### 3.3 相对增加或减少

使用 `+` 或 `-` 修改当前参数值：

```text
TKP+0.01\r\n
```

表示：

```text
new_track_kp = current_track_kp + 0.01
```

减少参数值：

```text
YKI-0.05\r\n
```

表示：

```text
new_yaw_ki = current_yaw_ki - 0.05
```

相对调整中的数值必须为非负数，最终 PID 参数必须位于 `0~100` 范围内。

### 3.4 停车命令

```text
STOP\r\n
```

该命令调用 `Car_Stop()`，清空控制状态并刹车。执行成功后，`cmd_status` 为 `1`，`mode` 为 `0`。

## 4. VOFA+ 使用流程

1. 打开 UART0 对应的 COM 口，配置为 115200、8N1、无流控。
2. 接收协议选择 `JustFloat`。
3. 按本文档顺序配置 15 个通道名称。
4. 通过 VOFA+ 的串口发送功能发送 ASCII 命令，并在命令末尾添加 `CRLF`。
5. 观察对应 PID 通道和 `cmd_status`，确认参数已经生效。

下位机不会返回文本形式的 `OK`。命令结果通过下一帧 JustFloat 中的 `cmd_status` 和 PID 参数通道反馈。

## 5. 示例

将循迹 Kp 增加 0.01：

```text
TKP+0.01\r\n
```

将航向 Kd 设置为 4.5：

```text
YKD=4.5\r\n
```

停车：

```text
STOP\r\n
```
