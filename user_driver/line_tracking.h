#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include <stdint.h>

/* 原灰度例程以低电平表示检测到黑线；若模块输出相反，只需改为1U。 */
#define LINE_ACTIVE_LEVEL          (0U)

/* 直行基础速度和最大转向修正量，单位均为motor_drive_percent使用的百分比。 */
#define TRACK_BASE_SPEED_PERCENT   (38)
#define TRACK_MAX_CORRECTION       (26)

/* 两路电机在车体“前进”时的符号；某侧反转时把对应的1改为-1。 */
#define LEFT_MOTOR_FORWARD_SIGN    (1)
#define RIGHT_MOTOR_FORWARD_SIGN   (1)

void Line_Tracking_Update(const uint16_t sensor_values[8]);

#endif /* LINE_TRACKING_H */
