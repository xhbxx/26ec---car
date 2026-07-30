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
| `main.c` | 传感器帧解析、20 ms 控制任务、双环调用、旋钮与串口目标值调整 |
| `user_driver/pid.c/.h` | 离散 PID、积分限幅、条件积分、积分衰减、微分滤波、初始化与复位 |
| `user_driver/Emm_V5.c/.h` | Emm_V5/X42S 串口协议和角度位置命令 |
| `user_driver/uart.c/.h` | CAM2串口读取、电机命令发送和PA31电机反馈读取 |
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

PA31 现在必须连接。程序每40 ms发送 `01 36 6B` 查询电机实时位置，并解析：

```text
Addr 36 Sign Position[4] 6B
```

Emm固件实时角度换算为 `Position * 360 / 65536`。首次有效反馈作为水管机械
零角，后续PID管角使用电机实际位置闭环校正。

### 3.3 旋钮编码器

| 编码器 | 地猛星 |
|---|---|
| A / CLK | PB24 |
| B / DT | PB20 |
| SW | PB19 |
| VCC | 3V3 |
| GND | GND |

旋转编码器可在 0~500 范围内调整 `target_position`，按下恢复为 250。
PB24、PB20、PB19 均为地猛星排针实际引出的 GPIO，输入已启用内部上拉；
旋钮模块的 `+` 接 3V3、`GND` 接地。

### 3.4 可选上位机目标位置接收

| 地猛星 | USB 转串口 |
|---|---|
| PA9 / UART1_RX | TX |
| GND | GND |

串口参数：115200，8N1，TTL 3.3 V 电平。程序不再向上位机发送CSV或运行
数据，因此PA8无需连接；PA9仅用于可选的 `SETPOINT:<value>` 接收。

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
TS/CS     平滑后的目标速度 / 按真实帧间隔计算的实际速度
A/M       PID目标管角 / 电机反馈实际管角，单位均为0.01度
P/I/D     位置外环参数，显示值为真实参数乘1000
```

运行时不再一次阻塞刷新整屏，而是在每个20 ms控制周期后只刷新一页，8个
周期完成整屏刷新，避免100 kHz I2C整屏传输破坏控制周期。

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

速度只在收到新的有效视觉帧时计算，并使用两帧真实毫秒间隔：

```text
raw_speed = (current_position - last_position) * 1000 / frame_elapsed_ms
```

没有新帧时保持上一次速度估计，不再人为插入0速度样本；随后使用一阶低通
滤波得到 `current_speed`。

### 5.1 位置外环

```text
position_error     = target_position - current_position
predicted_position = current_position + current_speed * 0.1s
target_speed       = Position_PID(target_position, predicted_position)
```

主要参数位于 `main.c` 顶部：

```c
POSITION_KP
POSITION_KI
POSITION_KD
POSITION_MAX_SPEED
POSITION_INTEGRAL_LIMIT
POSITION_DEADBAND
POSITION_DEADBAND_EXIT
POSITION_LOCK_SPEED
POSITION_LOOKAHEAD_S
POSITION_SLOW_ZONE
POSITION_NEAR_MIN_SPEED
POSITION_NEAR_SPEED_SLOPE
TARGET_SPEED_SLEW_PER_CYCLE
TARGET_SPEED_BRAKE_SLEW
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
PIPE_ANGLE_SLEW_PER_CYCLE
BALL_STOP_SPEED_THRESHOLD
```

当前管角限制为 ±4°。程序不再根据 `g_last_commanded_angle` 连续发送相对
增量，而是读取PA31实际角度并发送相对驱动器坐标零点的绝对位置目标：

```c
Emm_V5_Pos_Control_Angle(...);
```

当前机构靠近500一侧需要更大的回拉幅度，因此使用
往0方向对应负管角，`PIPE_NEGATIVE_ANGLE_GAIN` 当前为 `0.75`，用于减小该
方向的动作幅度；负方向还由 `PIPE_NEGATIVE_MAX_ANGLE_DEG=3°` 单独限幅。

目标角度与电机反馈实际角度相差小于0.12°时不重复发送，该阈值现在是实际
到位容差，不再是基于上一条软件指令的附加死区。电机命令至少间隔40 ms。

位置控制采用进入±1、退出±2的滞回死区：只有位置进入±1且球速绝对值不大于
2时才锁定。高速经过目标时不会被误认为已经到位，而会继续执行制动；误差超过
2后立即恢复缓慢调整。修改目标位置会立即解除旧目标锁定。

外环按当前速度预测约0.1秒后的球位置，用预测位置提前减速或反向制动。预测
结果被限制在-100~600，防止视觉速度尖峰产生极端控制量。进入目标附近80个位置
单位后，目标速度上限按 `1.5 + |位置误差| * 0.15` 动态下降，使小球靠近目标时仍能
缓慢微调。正常加速每20 ms最多变化2，减速或反向制动最多变化4，因此制动更
及时，但不会提高远距离正常追踪的加速幅度。

速度样本的有效时间单独限制为120 ms。超过120 ms没有新视觉速度样本时，程序立即
把旧速度置零，预测位置退回当前实际位置，防止小球停下后仍使用旧速度而在距离
目标约100的位置不再回调。位置帧整体丢失超过300 ms时仍执行原有安全停止。

当前位置环和速度环积分系数均为0，先排除旧积分造成的方向滞留。往0方向的负管
角增益为0.75，且该方向最大管角单独限制为3°；最低重新启动角仍保持0.25°，使
小球停在死区外时能够继续缓慢靠近目标。

为抑制机械惯性造成的大幅往返，当前采用保守控制基线：位置环Kp为0.30、最大
目标速度40，速度环Kp为0.040，全局最大管角4°，管角每20 ms最多变化0.15°。
当位置误差小于60时，最终管角进一步强制限制为±1.2°，因此目标附近只会小角度
加速或制动，不允许速度环因瞬时大速度误差而猛烈反打。

电机上电时的水管位置就是机械静止基准。程序收到第一帧有效电机位置反馈后，将
该绝对位置保存为 `g_motor_zero_absolute_angle`，后续PID角度和OLED的A/M都以
这个位置为0°。代码不再叠加固定平衡角，避免每次启动时在正确静止位置上额外
增加倾角。RESET前应让电机保持在需要作为静止基准的位置，零点采集后再开始控制。

视觉速度静止时存在约±10的位置单位/秒噪声。OLED的CS继续显示原始滤波速度，
但PID内部将±10范围按0处理，使目标附近静止的小球能够触发重新启动逻辑。

实测往500方向在A=0.37°~0.48°时不能启动，而A约1.19°可以启动；往0方向在
A=-0.29°时不能启动，而A约-0.91°可以启动。当前将静止重启角分别设置为
`PIPE_RESTART_POSITIVE_DEG=0.85°` 和 `PIPE_RESTART_NEGATIVE_DEG=1.20°`。
这些角度只在小球静止且仍位于目标外时使用，检测到运动后立即交回速度PID。

更换上电机械零点后，实测TS为负、A=-0.99°、M=-1.02°且球仍不动，说明新的
往0方向启动阈值已经超过1.02°。启动角检查现已移动到负方向增益之后，保证最终
送给电机的实际目标角至少达到-1.20°，不会再被0.75方向增益二次缩小。

C100到T250的首次过冲约11，而C400到T250会越过到C183，说明往500方向的制动
明显偏弱。因此目标60范围内的正管角上限提高到1.8°，负管角仍限制为1.2°；只
增强不够用的正向制动，不整体放大位置环或速度环。

电机反馈相对启动零点超出±15°时作为异常帧丢弃。当前控制命令只有±4°，该保护
用于避免串口错帧或位置符号跳变导致OLED的M突然显示999并参与闭环判断。

速度环积分恢复为 `SPEED_KI=0`，避免小球越过目标后仍保留旧方向积分而持续
往返摆动。小球在死区外已经静止且P输出不足时，改用
`PIPE_RESTART_MIN_ANGLE_DEG=0.25°` 的小角度启动补偿克服静摩擦。最终管角每20 ms
最多变化0.35°，用于限制机械动作幅度。

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

## 7. 上位机目标位置接收

程序不再向上位机发送 CSV、PID 参数或运行状态。UART1 只保留非阻塞接收；
如果需要从上位机修改目标位置，可以发送：

```text
SETPOINT:320\n
```

目标值自动限制在 0~500。错误指令会被静默丢弃，不阻塞控制循环。

PID 参数不通过该串口修改，需要直接在 `main.c` 开头手动调整。

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

### 小球来回震荡

- 先确认电机方向正确。
- 降低 `POSITION_KP` 或 `POSITION_MAX_SPEED`。
- 增加少量 `POSITION_KD`。
- 检查速度反馈噪声，必要时降低 `SPEED_FILTER_ALPHA`。
- 不要在速度内环未稳定前增加位置环积分。
