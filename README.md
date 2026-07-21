# MSPM0G3507 双路 TB6612 PWM 电机驱动

本工程已由原“一路直流电机 PID”示例改为双路 PWM 电机灰度循迹工程。

- 主循环读取八路数字灰度模块，并控制双路电机直行或差速转弯；
- 不使用原 PID 定时器闭环；
- 保留并补全原工程的 `motor_init()`、`motor_set_duty()`、`motor_set_direction()` 接口；
- 新增 `motor_drive_percent()` 和 `motor_stop()`，便于直接控制两路电机。

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
| PA14 | AD0 | 通道选择最低位 |
| PA15 | AD1 | 通道选择位1 |
| PA16 | AD2 | 通道选择最高位 |
| PA17 | OUT | 当前所选通道的数字输出 |
| 5V或模块规定电压 | VCC | 按模块说明供电 |
| GND | GND | 必须与开发板、TB6612和电机电源共地 |

注意：MSPM0G3507 GPIO不能直接接5V逻辑电平。若灰度模块OUT会输出5V，必须增加分压或3.3V电平转换后再接PA17。

传感器通道0～7应按车头左到右排列。代码默认低电平表示检测到黑线；如果实测相反，把 `user_driver/line_tracking.h` 中的 `LINE_ACTIVE_LEVEL` 改成 `1U`。

## 默认循迹动作

当前 `main.c` 循环执行：

```text
读取八路灰度 → 计算线的左右偏差 → 双路电机差速转弯
八路都没有检测到线 → 两路保持相同速度直线前行
```

基础速度在 `TRACK_BASE_SPEED_PERCENT` 修改，转弯强度上限在 `TRACK_MAX_CORRECTION` 修改。若某一侧实际方向相反，把 `LEFT_MOTOR_FORWARD_SIGN` 或 `RIGHT_MOTOR_FORWARD_SIGN` 从 `1` 改为 `-1`。

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
