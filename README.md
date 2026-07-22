## 26EC-CAR
# 设备清单<br>
  1.天猛星
  2.tb6612电机驱动
  3.张大头步进电机云台
  4.MPU6050
  5.八路灰度循迹
  6.Cam2
# 现已实现部分
  1.视觉追踪和云台追踪打靶心 (帧率太低，应用无法脱机运行)
  2.灰度循迹(不能拐弯)
# 下一步
  1.实现循迹完全版本

### 视觉云台 OLED部分接线表

请在断电状态下接线。CAM2、天猛星、两台 Emm42 和 OLED 的地线必须相连；CAM2 与天猛星仍可按比赛要求分别使用独立电源开关。

| 天猛星 MSPM0G3507 | 连接到 | 用途 |
|---|---|---|
| PA9 / UART1_RX | MaixCAM2 A21 / UART4_TX | 接收 CAM2 发送的视觉误差 |
| PA8 / UART1_TX | MaixCAM2 A22 / UART4_RX | MCU 回传线，当前程序预留，可不接 |
| PA21 / UART2_TX | 水平轴 Emm42 RX | 向水平轴发送控制命令 |
| PA22 / UART2_RX | 水平轴 Emm42 TX | 接收水平轴驱动器响应 |
| PA14 / UART3_TX | 俯仰轴 Emm42 RX | 向俯仰轴发送控制命令 |
| PA13 / UART3_RX | 俯仰轴 Emm42 TX | 接收俯仰轴驱动器响应 |
| PA1 / I2C0_SCL | OLED SCL（或 SCK） | OLED I2C 时钟 |
| PA0 / I2C0_SDA | OLED SDA | OLED I2C 数据 |
| 3V3 | OLED VCC | OLED 使用 3.3V 供电 |
| GND | CAM2 GND、两台 Emm42 GND、OLED GND | 所有通信设备必须共地 |
```

- 检测到黑线：根据八路灰度加权误差控制左右轮差速转弯。
- 八路都没有检测到黑线：左右轮保持相同速度直线前行。
- `encoder.c/.h` 已加入编译，左右编码器使用独立 GPIO 上升沿中断计数。
- `motor_drive_percent()` 接收目标百分比，`MOTOR_PID` 50 ms 定时器根据左右编码器脉冲分别修正两路 PWM。



## TB6612 接线

| MSPM0G3507 | TB6612 | 说明 |
|---|---|---|
| PB24 | STBY | 驱动器使能，电机初始化后拉高 |
| PA8 | AIN1 | A 路方向输入 1 |
| PA9 | AIN2 | A 路方向输入 2 |
| PA12 | PWMA | A 路 PWM |
| PA7 | BIN1 | B 路方向输入 1 |
| PB18 | BIN2 | B 路方向输入 2 |
| PA13 | PWMB | B 路 PWM |
| 3.3V | VCC | TB6612 逻辑电源 |
| GND | GND | 与开发板、电机电源负极共地 |

`VM` 接电机独立电源正极，`AO1/AO2` 接电机 A，`BO1/BO2` 接电机 B。电机电源不要接到开发板 3.3V。开发板 GND、TB6612 GND 和电机电源负极必须连接在一起。

## 八路灰度模块接线

| MSPM0G3507 | 灰度模块 | 说明 |
|---|---|---|
| PA14 | AD0 | 通道选择位 0 |
| PA15 | AD1 | 通道选择位 1 |
| PA16 | AD2 | 通道选择位 2 |
| PA17 | OUT | 当前通道数字输出 |
| 模块规定电压 | VCC | 按模块规格供电 |
| GND | GND | 与开发板和电机系统共地 |

传感器通道 0 到 7 应按车头左侧到右侧排列。代码默认低电平表示检测到黑线；若模块逻辑相反，将 `user_driver/line_tracking.h` 中的 `LINE_ACTIVE_LEVEL` 改为 `1U`。

MSPM0G3507 GPIO 不能直接输入 5V。若灰度模块 `OUT` 是 5V 电平，需先分压或使用 3.3V 电平转换。

## 双路编码器接线和 PID

| 编码器 | MSPM0G3507 | 说明 |
|---|---|---|
| 左轮编码器 A 相 | PA22 | GPIOA 上升沿中断计数 |
| 右轮编码器 A 相 | PB9 | GPIOB 上升沿中断计数 |
| 两路编码器 VCC | 3.3V | 编码器逻辑电源 |
| 两路编码器 GND | GND | 与开发板、TB6612和电机电源负极共地 |

当前闭环只使用编码器 A 相，B 相暂不接入；电机方向仍由 TB6612 方向引脚决定。编码器输出如果是 5V，必须先通过分压或电平转换后再接 PA22/PB9。

PID 参数位于 `user_driver/motor.h`：

```c
#define MOTOR_ENCODER_PULSES_AT_100 (20)  // 100%速度每50 ms的目标脉冲数
#define MOTOR_PID_KP                (12)
#define MOTOR_PID_KI                (2)
#define MOTOR_PID_KD                (1)
```

`MOTOR_ENCODER_PULSES_AT_100` 需要按实际编码器和减速比标定。测量电机在 100% PWM 下 50 ms 内的 A 相脉冲数，再将结果填入该宏。

## 方框循迹和直角转弯参数

方框循迹使用非阻塞状态机：正常差速循迹、短距离越过拐点、原地 90 度转弯，直到中间灰度探头重新检测到下一条边后恢复循迹。参数位于 `user_driver/line_tracking.h`：

```c
#define TRACK_SQUARE_TURN_DIRECTION  (1)   // 1顺时针右转，-1逆时针左转
#define TRACK_CORNER_TURN_SPEED      (24)  // 原地转弯速度百分比
#define TRACK_CORNER_ADVANCE_UPDATES (4U)  // 检测到拐点后继续前进的循环次数
#define TRACK_TURN_MIN_UPDATES       (12U) // 转弯后允许重新抓线的最短时间
```

如果车头还没到直角顶点就开始转，增大 `TRACK_CORNER_ADVANCE_UPDATES`；如果越过顶点太多，减小该值。转弯过快或冲过下一条边时降低 `TRACK_CORNER_TURN_SPEED`。

## OLED 和调试串口预留接线

| 功能 | MSPM0G3507 | 说明 |
|---|---|---|
| OLED SCL | PA1 | I2C0 时钟 |
| OLED SDA | PA0 | I2C0 数据 |
| 串口 TX | PA28 | 115200 波特率，接 USB 转串口 RX |
| 串口 RX | PA31 | 115200 波特率，接 USB 转串口 TX |
| GND | GND | 与外设共地 |

当前 `main.c` 使用已验证的 `oled.c/.h` 初始化屏幕，并约每 500 ms 刷新一次：`L` 显示左轮最近 50 ms 的编码器脉冲数，`R` 显示右轮最近 50 ms 的编码器脉冲数。OLED 读取的是快照，不会清零或抢走 PID 使用的计数。

PA0/PA1 的 I2C 开漏模式未启用芯片内部上拉。常见四针 OLED 模块已自带上拉；若模块未带上拉电阻，需要从 SDA、SCL 各接约 4.7 kΩ 到 3.3V。

## 参数调整位置

循迹参数位于 `user_driver/line_tracking.h`：

```c
#define LINE_ACTIVE_LEVEL          (0U)  // 黑线有效电平
#define TRACK_BASE_SPEED_PERCENT   (38)  // 直行基础速度
#define TRACK_MAX_CORRECTION       (26)  // 最大转向修正量
#define LEFT_MOTOR_FORWARD_SIGN    (1)   // 左电机前进方向
#define RIGHT_MOTOR_FORWARD_SIGN   (1)   // 右电机前进方向
```

若某侧电机方向相反，只修改对应的 `*_FORWARD_SIGN` 为 `-1`。电机最终速度范围由 `motor_drive_percent()` 限制在 `-100%` 到 `100%`。

## 电机接口

```c
motor_init(MOTOR_ID_A);
motor_init(MOTOR_ID_B);

motor_drive_percent(MOTOR_ID_A, 30);   // A 路正转，30% 占空比
motor_drive_percent(MOTOR_ID_B, -30);  // B 路反转，30% 占空比

motor_stop(MOTOR_ID_A);
motor_stop(MOTOR_ID_B);
```

所有初始化应只在 `main()` 进入主循环前执行一次，不要在循环中重复初始化电机或外设。

## CCS 打开和编译

1. 在 CCS 中选择 **File -> Import Projects from File System**。
2. 选择本目录 `10_DC_MOTOR_PID_3`，导入后 CCS 中显示的工程名为 `G3507`。
3. 双击并保存 `empty.syscfg`，让 SysConfig 重新生成配置文件。
4. 执行 **Project -> Clean**，再执行 **Build Project**。
5. 正常输出文件为 `Debug/G3507.out`。

若曾在 CCS 中导入旧工程 `10_DC_MOTOR_PID_2`，先从 CCS 工作区移除旧项目引用，但不要勾选删除磁盘内容，再重新导入本目录。由于目录当前被其他程序占用，磁盘文件夹暂时仍叫 `10_DC_MOTOR_PID_3`；CCS 工程名和输出名已经改为 `G3507`。
