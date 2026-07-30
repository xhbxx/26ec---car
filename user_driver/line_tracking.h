#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include <stdint.h>

/* 正常循迹速度和差速修正上限，单位为 motor_drive_percent() 百分比。 */
#define TRACK_REFERENCE_BASE_PERCENT     (30)// 中间直行基础速度
#define TRACK_MIN_FORWARD_PERCENT        (10)// 转弯时最低前进速度
#define TRACK_MAX_CORRECTION_PERCENT      (20)// 最大左右差速
#define TRACK_EDGE_SLOWDOWN_PERCENT       (10)// 到最外侧时基础速度降低10%

/* ATK-MS6DSV Z轴角速度校准和丢线直行修正参数。 */
#define TRACK_GYRO_CALIBRATION_SAMPLES    (200U)
#define TRACK_GYRO_DEADBAND_RAW           (100L)  /* 约1度/秒，量程为±2000 dps */
#define TRACK_GYRO_CORRECTION_SIGN        (-1)
#define TRACK_GYRO_SINGLE_WHEEL_BOOST_PERCENT (2)

/* 两路电机安装方向；某侧前进方向相反时将对应值改为 -1。 */
#define LEFT_MOTOR_FORWARD_SIGN        (-1)
#define RIGHT_MOTOR_FORWARD_SIGN       (1)

uint8_t Line_Tracking_Update(const uint16_t sensor_values[8]);
int8_t Line_Tracking_GetGyroDirection(void);

#endif /* LINE_TRACKING_H */
