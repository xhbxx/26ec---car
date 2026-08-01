#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#include <stdint.h>

/* ==================== 场地尺寸（单位：mm） ==================== */
/* AB 第一段直线目标距离。 */
#define TRACK_STRAIGHT_MM                 (1510L)
/* Mode 2 的 CD 第二段直线目标距离，单独修改不会影响 Mode 5。 */
#define TASK2_CD_STRAIGHT_MM              (1455L)
/* Mode 5 的 CD 第二段直线目标距离，单独修改不会影响 Mode 2。 */
#define TASK5_CD_STRAIGHT_MM              (1440L)
/* 赛题圆弧中心线半径，通常按题目固定，不建议用它补偿转向误差。 */
#define TRACK_CURVE_RADIUS_MM             (500L)
/* 半圆弧中心线长度；500 mm 半径对应约 1571 mm。 */
#define TRACK_HALF_CURVE_MM               (1571L)
/* 黑色引导线宽度，仅用于记录场地参数。 */
#define TRACK_LINE_WIDTH_MM               (18L)
/* 停车允许距离误差，仅用于记录允许范围。 */
#define TRACK_STOP_TOLERANCE_MM           (20L)
/* Mode 2/5 整圈长度分别跟随各自的 CD 直线距离。 */
#define TASK2_LAP_LENGTH_MM               \
    (TRACK_STRAIGHT_MM + TASK2_CD_STRAIGHT_MM + \
        2L * TRACK_HALF_CURVE_MM)
#define TASK5_LAP_LENGTH_MM               \
    (TRACK_STRAIGHT_MM + TASK5_CD_STRAIGHT_MM + \
        2L * TRACK_HALF_CURVE_MM)

/* ==================== 底盘尺寸与路径切换 ==================== */
/* 左右驱动轮中心距；应填写实测值，影响理论圆弧轮速比。 */
#define CHASSIS_TRACK_WIDTH_MM            (200L)
/* 整圈检测不到终点标志时的距离兜底余量。 */
#define LAP_FALLBACK_EXTRA_MM             (100L)

/* ==================== 电机安装方向 ==================== */
/* 左轮正向符号；左轮实际向后转时只在 -1 和 1 之间反转。 */
#define CHASSIS_LEFT_FORWARD_SIGN         (-1)
/* 右轮正向符号；右轮实际向后转时只在 -1 和 1 之间反转。 */
#define CHASSIS_RIGHT_FORWARD_SIGN        (1)

/* ==================== 各模式速度（单位：mm/s） ==================== */
/* Mode 2 直线速度；增大可缩短时间，但钢球更容易移动。 */
#define TASK2_STRAIGHT_MMPS               (450)
/* Mode 2 第一个弯道中心速度；过大会冲出轨迹。 */
#define TASK2_CURVE_MMPS                  (450)
/* Mode 2 第二个弯道中心速度；与第一弯分开调，当前提高 20 mm/s。 */
#define TASK2_SECOND_CURVE_MMPS           (475)
/* Mode 2 第一个弯道基础半差速；增大转弯更急，减小转弯更缓。 */
#define TASK2_CURVE_STEERING_MMPS         (140L)
/* Mode 2 第二个弯道基础半差速；与第一弯分开调，当前增加 15 mm/s。 */
#define TASK2_SECOND_CURVE_STEERING_MMPS  (155L)
/* Mode 2 弯道陀螺仪角度比例和最大追加半差速。 */
#define TASK2_GYRO_CURVE_KP_MMPS_PER_DEG  (8L)
#define TASK2_GYRO_CURVE_LIMIT_MMPS       (40L)
/* Mode 2 直线陀螺仪角度比例和最大修正半差速。 */
#define TASK2_GYRO_STRAIGHT_KP_MMPS_PER_DEG (5L)
#define TASK2_GYRO_STRAIGHT_LIMIT_MMPS    (30L)
/* Mode 2 主动短刹车的估算减速度和位置余量，用于提前预测停车点。 */
#define TASK2_ACTIVE_BRAKE_DECEL_MMPS2    (500L)
#define TASK2_BRAKE_MARGIN_MM             (10L)
/* Mode 4 直线速度。 */
#define TASK4_STRAIGHT_MMPS               (220)
/* Mode 5 直线速度。 */
#define TASK5_STRAIGHT_MMPS               (270)
/* Mode 5 弯道速度。 */
#define TASK5_CURVE_MMPS                  (230)
/* Mode 4/5 直线陀螺仪修正；增大比例可更快纠正偏航。 */
#define TASK45_GYRO_STRAIGHT_KP_MMPS_PER_DEG (3L)
#define TASK45_GYRO_STRAIGHT_LIMIT_MMPS   (25L)
/* Mode 5 编码器圆弧基础半差速与陀螺仪修正。 */
#define TASK5_CURVE_STEERING_MMPS         (40L)
#define TASK5_GYRO_CURVE_KP_MMPS_PER_DEG  (2L)
#define TASK5_GYRO_CURVE_LIMIT_MMPS       (20L)
/* 最终接近停车点时的速度；减小可提高停车精度。 */
#define FINAL_APPROACH_MMPS               (120)

/* ==================== 加减速度（单位：mm/s^2） ==================== */
/* 数值越小启动越柔和，但达到目标速度所需时间越长。 */
#define TASK4_ACCEL_MMPS2                 (95L)  /* Mode 4 加速度。 */
#define TASK4_DECEL_MMPS2                 (80L)  /* Mode 4 减速度。 */
#define TASK5_ACCEL_MMPS2                 (55L)  /* Mode 5 加速度。 */
#define TASK5_DECEL_MMPS2                 (70L)  /* Mode 5 减速度。 */

/* ==================== 四路红外循迹 ==================== */
/* 红外传感器实际数量；当前硬件为四路，不要改成五路。 */
#define LINE_SENSOR_COUNT                 (4U)
/* 转向差速变化速度；减小更平滑，增大则弯道响应更快。 */
#define LINE_STEERING_SLEW_MMPS2          (200L)
/* 连续检测到终点黑线的次数；增大可抗干扰，但停车响应稍慢。 */
#define LINE_MARKER_STABLE_SAMPLES        (3U)

/* ==================== ATK-MS6DSV 陀螺仪 ==================== */
/* 陀螺仪读取周期；越小更新越快，但总线和 CPU 占用更高。 */
#define IMU_UPDATE_PERIOD_MS              (10UL)
/* 上电静止零偏采样数；增大更稳定但启动等待更久，80 约为 0.8 秒。 */
#define IMU_CALIBRATION_SAMPLES           (40U)
/* 原始角速度死区；增大可减少静止漂移，过大会忽略慢速转动。 */
#define IMU_GYRO_DEADBAND_RAW             (20L)
/* 角度方向符号；顺时针转动车体时 OLED 应显示正值，否则反转此符号。 */
#define GYRO_Z_CLOCKWISE_SIGN             (1L)
/* 每个半圆的目标转角，单位为千分之一度；180000 表示 180 度。 */
#define CURVE_TARGET_ANGLE_MDEG           (180000L)

/* ==================== 控制周期和任务超时 ==================== */
/* 底盘控制周期；通常保持 5 ms，不建议随意增大。 */
#define CONTROL_UPDATE_PERIOD_MS          (5UL)
/* OLED 刷新周期；增大可降低刷新占用，但显示变化会更慢。 */
#define DISPLAY_UPDATE_PERIOD_MS          (150UL)
/* Mode 2 最长运行时间，略大于赛题限制用于故障保护。 */
#define TASK2_TIMEOUT_MS                  (20000UL)
/* Mode 4/5 最长运行时间。 */
#define TASK4_TIMEOUT_MS                  (8000UL)
#define TASK5_TIMEOUT_MS                  (30000UL)

#endif /* CAR_CONFIG_H */
