# 26EC 循迹小车

# 已实现
    1.灰度循迹及其逻辑
    2.电机驱动+PID
# 待修改
    1.调整pid
    2.硬件板子有点问题

## 按键 
| MSPM0G3507 | PID action | Button wiring |
|---|---|---|
| PA23 | Kp + 1 | Press connects PA23 to GND |
| PA21 | Kp - 1 | Press connects PA21 to GND |
| PA18 | Ki + 1 | Press connects PA18 to GND |
| PA17 | Ki - 1 | Press connects PA17 to GND |

The four inputs use internal pull-up resistors and are active low. Each press changes one step;
holding a key does not repeat until it is released. Kp and Ki are limited to
0..100 and are shown on OLED row 3.

## 其余模块接线

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

## TB6612 接线

| MSPM0G3507 | TB6612 | 说明 |
|---|---|---|
| PB24 | STBY | 驱动器待机使能，程序初始化后输出高电平 |
| PA8 | AIN1 | 电机A方向输入1 |
| PA9 | AIN2 | 电机A方向输入2 |
| PA12 | PWMA | 电机A PWM |
| PA7 | BIN1 | 电机B方向输入1 |
| PB18 | BIN2 | 电机B方向输入2 |
| PA13 | PWMB | 电机B PWM |
| 3V3 | VCC | TB6612逻辑电源 |
| GND | GND | 必须与电机电源、开发板共地 |

TB6612 的 `VM` 接电机独立电源正极；`AO1/AO2` 接电机A两端；`BO1/BO2` 接电机B两端。不要将电机供电接到开发板3.3V。

## 八路灰度模块接线

| MSPM0G3507 | 八路灰度模块 | 说明 |
|---|---|---|
| PB12 | AD0 | 通道选择最低位 |
| PB04 | AD1 | 通道选择位1 |
| PB05 | AD2 | 通道选择最高位 |
| PB13 | OUT | 当前所选通道的数字输出 |
| 5V或模块规定电压 | VCC | 按模块说明供电 |
| GND | GND | 必须与开发板、TB6612和电机电源共地 |

注意：MSPM0G3507 GPIO不能直接接5V逻辑电平。若灰度模块OUT会输出5V，必须增加分压或3.3V电平转换后再接PA17。

## 编码器、ADC和PID

| MSPM0G3507 | 信号 | 用途 |
|---|---|---|
| PB8 | 左编码器脉冲输出 | GPIO上升沿中断计数，对应OLED的LA实际速度 |
| PB9 | 右编码器输出 | GPIO数字采样，避免占用灰度和ADC引脚 |
| GND | 编码器地 | 必须共地，编码器输出不得超过3.3V |

当前G3507封装的ADC0实例只生成一个外部通道，因此左轮使用ADC采样，右轮使用GPIO采样；两路都在 `Encoder_Sample_ADC()` 中转成上升沿计数。若你的编码器是标准3.3V方波，直接使用GPIO采样更可靠；PA22需要接模拟/脉冲输出并经过限压。

PID参数在 `user_driver/motor.h` 的 `MOTOR_PID_KP`、`MOTOR_PID_KI`、`MOTOR_PID_KD` 修改。闭环在 `user_driver/motor.c` 的 `MOTOR_PID_INST_IRQHandler()` 中运行，周期由 `empty.syscfg` 的 `MOTOR_PID` 定时器决定（当前50 ms）。

串口输出在 `main.c`，约100 ms发送一帧：

```text
PID,P=1234,I=56,D=-78
```

三个数值均放大1000倍，分别是左轮最近一次PID的P、I、D项；串口为 `PRINT`，115200 baud，PA28=TX、PA31=RX。

传感器通道0～7应按车头左到右排列。代码默认低电平表示检测到黑线；如果实测相反，把 `user_driver/line_tracking.h` 中的 `LINE_ACTIVE_LEVEL` 改成 `1U`。

## 默认循迹动作

当前 `main.c` 循环执行：

```text
读取八路灰度 → 计算线的左右偏差 → 双路电机差速转弯
八路都没有检测到线 → 两路保持相同速度直线前行
```

基础速度在 `TRACK_BASE_SPEED_PERCENT` 修改，转弯强度上限在 `TRACK_MAX_CORRECTION` 修改。若某一侧实际方向相反，把 `LEFT_MOTOR_FORWARD_SIGN` 或 `RIGHT_MOTOR_FORWARD_SIGN` 从 `1` 改为 `-1`。速度百分比的最终限幅在 `motor_drive_percent()` 内。

## 驱动接口

```c
motor_init(MOTOR_ID_A);
motor_init(MOTOR_ID_B);

motor_drive_percent(MOTOR_ID_A, 30);   // A正转，30%占空比
motor_drive_percent(MOTOR_ID_B, -30);  // B反转，30%占空比

motor_set_duty(MOTOR_ID_A, 2000U);     // A设为50%占空比，范围0~4000
motor_set_direction(MOTOR_ID_A, MOTOR_DIRECTION_FORWARD);

motor_stop(MOTOR_ID_A);                // A停止并短刹车
motor_stop(MOTOR_ID_B);                // B停止并短刹车
```

## CCS 使用

1. 在 CCS 中打开本目录工程。
2. 双击 `empty.syscfg` 后保存，让 SysConfig 生成新的 `ti_msp_dl_config.c/.h`，以便产生电机和 `GRAYSCALE` 引脚定义。
3. 执行 **Project → Clean**，再 **Build Project** 和烧录。

第一次通电请让两路电机空载，确认TB6612供电、电机线和共地正确后再连接机构。

## MPU6050 接线与显示

| MPU6050 | MSPM0G3507 | 说明 |
|---|---|---|
| VCC | 3.3V | 不要接5V逻辑电平 |
| GND | GND | 必须与主控、电机驱动共地 |
| SDA | PB3 | I2C1数据线 |
| SCL | PB2 | I2C1时钟线 |
| INT | PB1 | 数据就绪上升沿中断 |
| AD0 | GND | 设备地址保持为 `0x68` |

`SDA` 和 `SCL` 需要上拉到3.3V。大多数MPU6050模块已带上拉电阻；若模块没有，两根线分别外接约4.7kΩ到3.3V。

OLED显示字段：

```text
L / R : 左右轮编码器实际速度百分比
X / Y : MPU6050加速度X/Y轴原始值
G     : MPU6050陀螺仪Z轴原始值
ON:1  : WHO_AM_I检测成功
ON:0  : 未识别到MPU6050
```

`oled.c` 只支持12、16、24号英文字体，MPU6050数值必须使用12号字体，使用8号字体会直接不显示。

当8路灰度均未检测到黑线时，`line_tracking.c` 直接给左右轮相同的
`TRACK_REFERENCE_BASE_PERCENT` 目标速度继续直行，不再使用陀螺仪差速，
也不会保持丢线前的转弯动作。

OLED第三行右侧现在显示 `S:n W:nnn`：

- `S:0 W:000`：没有收到设备响应。检查3.3V、共地、PB3/PB2接线和I2C上拉。
- `S:1 W:104`：通信及数据读取正常，十进制104就是 `WHO_AM_I=0x68`。
- `S:2 W:104`：已经识别设备，但连续传感器数据读取失败。
- `S:3 W:104`：已经识别设备，但配置寄存器写入失败。

驱动会自动探测 `0x68` 和 `0x69`，上电识别失败或连续读取失败后会自动重连。
只有 `S:1` 时数据才有效。模块水平静止时，Z轴加速度应接近 `16384`，不应三轴全为0。

## OLED和GY-6500最终接线

| 模块 | MSPM0G3507 |
|---|---|
| OLED SDA | PA0 / I2C0_SDA |
| OLED SCL | PA1 / I2C0_SCL |
| OLED VCC | 3.3V |
| OLED GND | GND |
| GY-6500 SDA | PB3 / I2C1_SDA |
| GY-6500 SCL | PB2 / I2C1_SCL |
| GY-6500 INT | PB1，可不接 |
| GY-6500 AD0 | GND，地址固定为0x68 |
| GY-6500 CS/NCS | 3.3V，仅模块引出该引脚时需要 |

PA0、PA1作为硬件I2C开漏引脚时SysConfig不允许启用内部上拉。多数四针OLED模块
已经自带上拉，可以直接连接；如果模块没有上拉，SDA、SCL必须各使用一个约4.7kΩ
电阻上拉到3.3V，I2C总线不能在完全没有上拉的情况下工作。

### GY-6500说明

当前驱动已支持GY-6500上的MPU6500芯片。GY-6500正常时OLED应显示
`S:1 W:112`，其中112是 `WHO_AM_I=0x70` 的十进制值。驱动同时保留
MPU6050兼容性，因此 `S:1 W:104` 也会被识别。两者的原始加速度和
陀螺仪寄存器地址相同，现有循迹调用接口无需更换。

## 电机速度显示与保护

OLED前两行显示：`LT`/`RT`为左右轮设定速度百分比，`LA`/`RA`为左右轮
编码器测得的真实速度百分比。正负号表示方向。PID输出上限为当前目标百分比加
`MOTOR_PID_DUTY_HEADROOM_PERCENT`，默认余量15%。例如目标40%时PWM最多55%，
即使编码器断线或持续读到0，也不会继续积分到100%占空比。

## GY-6500丢线直行

GY-6500只使用车体竖直方向的Z轴陀螺仪。上电后保持小车静止，程序采集200个样本
计算零偏，校准期间电机保持停止。OLED显示：`G`为Z轴原始值，`B`为静止零偏，
`E=G-B`为去零偏后的转动量，`C`为最终左右轮差速修正百分比，`R1`表示校准完成。
检测到黑线时仍使用灰度位置权重；八路全0时才使用GY-6500抑制转动。

参数位于 `user_driver/line_tracking.h`：死区默认65原始单位，300原始单位转换为1%
差速，最大修正4%。若修正方向相反，将 `TRACK_GYRO_CORRECTION_SIGN` 从1改为-1。

当前检测到黑线时能够正常循迹，因此左右电机方向均保持原配置 `+1`。丢线转圈属于
陀螺仪修正方向形成正反馈，当前 `TRACK_GYRO_CORRECTION_SIGN` 已设为 `-1`。
