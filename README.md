# MSPM0G3507 双环 PID 水管小球控制系统

## 1. 工程说明

工程目录：

```text
E:\install\keli\NEDC\10_DC_MOTOR_PID_3
```

MCU 为 TI MSPM0G3507，使用 CCS、MSPM0 SDK 2.10.0.04 和 SysConfig
1.26.2。系统通过视觉位置反馈控制 Emm_V5/X42S 闭环步进电机改变水管倾角，
使小球稳定在目标位置。

当前控制结构：

```text
target_position
      ↓
位置 PID（位置误差 → target_speed）
      ↓
速度 PID（速度误差 → pipe_angle）
      ↓
Emm_V5 位置角度命令
      ↓
水管倾角与小球运动
      ↓
current_position
```

## 2. 当前文件结构

| 文件 | 作用 |
|---|---|
| `main.c` | 传感器帧解析、20 ms 控制任务、双环调用、目标值调整、llm-pid-tuner 通信 |
| `user_driver/pid.c/.h` | 离散 PID、积分限幅、输出限幅、微分滤波、初始化与复位 |
| `user_driver/Emm_V5.c/.h` | Emm_V5/X42S 串口协议和角度位置命令 |
| `user_driver/uart.c/.h` | 传感器串口读取和电机串口发送 |
| `user_driver/bianma.c/.h` | 旋钮编码器读取 |
| `user_driver/delay.c/.h` | 启动阶段毫秒延时 |
| `empty.syscfg` | GPIO 与三路 UART 配置 |

## 3. 接线

所有模块必须共地。电机使用独立动力电源，不能由开发板 3.3 V 引脚供电。

### 3.1 位置传感器/CAM2

串口参数：115200，8N1。

| 地猛星 | 传感器/CAM2 | 说明 |
|---|---|---|
| PA22 / UART2_RX | TX | 接收小球位置帧 |
| PA21 / UART2_TX | RX | 当前不需要回传时可不接 |
| GND | GND | 必须共地 |

### 3.2 Emm_V5/X42S 电机

这组引脚已经过实际转动测试：

| 地猛星 | Emm_V5/X42S TTL 接口 |
|---|---|
| PA28 / UART0_TX | 驱动器 RX（R/A/H） |
| PA31 / UART0_RX | 驱动器 TX（T/B/L） |
| GND | 驱动器信号 GND |

驱动器菜单需要确认：

```text
P_Serial = UART_FUN
UartBaud = 115200
Checksum = 0x6B
FWType   = FW_Emm
Motor ID = 1
```

当前为 TTL 串口连接。RS485 型号不能直接连接 MCU UART，必须增加 RS485
收发器。

### 3.3 旋钮编码器

| 编码器 | 地猛星 |
|---|---|
| A / CLK | PB24 |
| B / DT | PB20 |
| SW | PB19 |
| VCC | 3V3 |
| GND | GND |

旋转编码器可在 0~500 范围内调整 `target_position`，按下恢复为 240。
PB24、PB20、PB19 均为地猛星排针实际引出的 GPIO，输入已启用内部上拉；
旋钮模块的 `+` 接 3V3、`GND` 接地。

### 3.4 llm-pid-tuner/USB 转串口

| 地猛星 | USB 转串口 |
|---|---|
| PA8 / UART1_TX | RX |
| PA9 / UART1_RX | TX |
| GND | GND |

串口参数：115200，8N1，TTL 3.3 V 电平。

### 3.5 0.96英寸四针 OLED

| OLED | 地猛星 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | PA0 / I2C0_SDA |
| SCL | PA1 / I2C0_SCL |

OLED 地址为 `0x3C`，每250 ms刷新一次：

```text
T/C       目标位置 / 当前位置，V表示反馈有效，X表示反馈丢失
TS/CS     目标速度 / 滤波后的实际速度
A         管角，显示单位为0.01度
P/I/D     位置外环参数，显示值为真实参数乘1000
```

## 4. 传感器数据协议

位置数据为固定 7 字节二进制帧：

```text
AA 55 01 SEQ POS_L POS_H CRC8
```

位置采用小端格式：

```c
current_position = POS_L | (POS_H << 8);
```

要求：

- 有效位置范围为 0~500。
- CRC 为 CRC-8/ATM，多项式 `0x07`，初值 `0x00`。
- CRC 计算范围为前 6 字节。
- 超过 300 ms 未收到有效位置帧，程序停止电机并复位两个 PID。

## 5. 双环 PID

控制周期固定为 20 ms：

```c
#define CONTROL_PERIOD_MS (20UL)
#define CONTROL_DT_S      (0.020f)
```

速度计算：

```text
raw_speed = (current_position - last_position) / 0.02
```

随后使用一阶低通滤波得到 `current_speed`。

### 5.1 位置外环

```text
position_error = target_position - current_position
target_speed   = Position_PID(target_position, current_position)
```

主要参数位于 `main.c` 顶部：

```c
POSITION_KP
POSITION_KI
POSITION_KD
POSITION_MAX_SPEED
POSITION_INTEGRAL_LIMIT
POSITION_DEADBAND
```

### 5.2 速度内环

```text
speed_error = target_speed - current_speed
pipe_angle = Speed_PID(target_speed, current_speed)
```

主要参数：

```c
SPEED_KP
SPEED_KI
SPEED_KD
SPEED_INTEGRAL_LIMIT
SPEED_FILTER_ALPHA
PIPE_MAX_ANGLE_DEG
```

当前管角限制为 ±10°。`Motor_SetAngle()` 将目标管角转换为相邻周期的相对
角度增量，再调用已有的：

```c
Emm_V5_Pos_Control_Angle(...);
```

小于 0.12° 的变化暂不发送，避免电机频繁抖动。

## 6. PID 调试顺序

### 6.1 先调速度内环

将：

```c
#define POSITION_LOOP_ENABLED (1U)
```

改为：

```c
#define POSITION_LOOP_ENABLED (0U)
```

此时使用 `SPEED_LOOP_TEST_TARGET` 作为固定目标速度。

推荐步骤：

1. 令 `SPEED_KI=0`、`SPEED_KD=0`。
2. 从较小值逐渐增加 `SPEED_KP`。
3. 出现持续震荡后适当降低 `SPEED_KP`。
4. 少量增加 `SPEED_KI`，消除长期速度偏差。
5. 必要时少量增加 `SPEED_KD`，抑制速度突变和过冲。

### 6.2 再调位置外环

速度环稳定后将 `POSITION_LOOP_ENABLED` 恢复为 `1U`。

1. 先令 `POSITION_KI=0`，从小到大调整 `POSITION_KP`。
2. 增加 `POSITION_KD` 减少过冲和往返摆动。
3. 最后只加入很小的 `POSITION_KI` 消除稳态位置误差。

参数过大通常表现为震荡、冲过目标和电机频繁动作；参数过小通常表现为
响应慢、到达目标时间长。

## 7. llm-pid-tuner 通信

程序每个 20 ms 控制周期发送一行 CSV，不发送标题行：

```text
timestamp_ms,setpoint,input,pwm,error,p,i,d
```

示例：

```text
1520,240,218,2.35,22,0.35,0.00,0.10
```

字段映射：

| 字段 | 当前工程变量 |
|---|---|
| timestamp_ms | `g_milliseconds` |
| setpoint | `target_position` |
| input | `current_position` |
| pwm | `pipe_angle`，单位为度 |
| error | `target_position - current_position` |
| p/i/d | 外环 `g_position_pid.Kp/Ki/Kd` |

PC 修改目标值时发送：

```text
SETPOINT:320\n
```

目标值自动限制在 0~500。错误指令会被静默丢弃，不阻塞控制循环。

注意：当前版本只支持 llm-pid-tuner 修改目标值。CSV 中的 P/I/D 是位置外环
参数；内环速度 PID 参数仍需在代码中手动维护，当前也没有实现通过串口直接
修改 P/I/D 的命令。

## 8. 电机方向与角度参数

如果电机动作使位置误差继续增大，交换：

```c
#define EMM_DIRECTION_POSITIVE (0U)
#define EMM_DIRECTION_NEGATIVE (1U)
```

电机步距角和细分设置位于 `user_driver/Emm_V5.h`：

```c
#define EMM_V5_STEP_ANGLE_X10_DEG (18U)
#define EMM_V5_MICROSTEP          (16U)
```

它们必须与驱动器实际设置一致，否则命令角度与真实转角不一致。

## 9. CCS 编译和烧录

1. 在 CCS 中导入 `E:\install\keli\NEDC\10_DC_MOTOR_PID_3`。
2. 确认 MSPM0 SDK 为 2.10.0.04，SysConfig 为 1.26.2。
3. 保存 `empty.syscfg`。
4. 执行 `Project -> Clean Project`。
5. 执行 `Project -> Build Project`。
6. 烧录 `Debug\10_DC_MOTOR_PID_2.out`。

## 10. 常见问题

### 电机不转

- 确认 PA28 TX 接驱动器 RX，不要再接旧版 PA21。
- 确认驱动器与地猛星共地。
- 确认驱动器使用独立动力电源。
- 检查 `UART_FUN`、115200、`Checksum=0x6B`、`FW_Emm` 和地址 1。
- 检查驱动器是否处于欠压、堵转、过流或过热保护状态。

### current_position 不更新

- 确认传感器 TX 接 PA22，而不是 PA31。
- 确认两端波特率均为 115200。
- 确认发送的是 7 字节二进制帧，不是 ASCII 数字。
- 检查 CRC-8/ATM 和 0~500 范围。

### llm-pid-tuner 收不到 CSV

- 确认 PA8 TX 接 USB 转串口 RX。
- 确认 GND 共地且使用 3.3 V TTL 电平。
- 串口工具设置为 115200、8N1。
- CSV 只有在 MCU 主循环正常运行时持续发送。

### 小球来回震荡

- 先确认电机方向正确。
- 降低 `POSITION_KP` 或 `POSITION_MAX_SPEED`。
- 增加少量 `POSITION_KD`。
- 检查速度反馈噪声，必要时降低 `SPEED_FILTER_ALPHA`。
- 不要在速度内环未稳定前增加位置环积分。
