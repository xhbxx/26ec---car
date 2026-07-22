#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include <stdint.h>

/* 灰度模块默认低电平表示检测到黑线；模块逻辑相反时改为 1U。 */
#define LINE_ACTIVE_LEVEL              (0U)

/* 正常循迹速度和差速修正上限，单位为 motor_drive_percent() 百分比。 */
#define TRACK_BASE_SPEED_PERCENT       (60)
#define TRACK_MAX_CORRECTION           (60)

/* 方框转弯方向：1 为连续右转（顺时针），-1 为连续左转（逆时针）。 */
#define TRACK_SQUARE_TURN_DIRECTION    (1)
#define TRACK_CORNER_TURN_SPEED        (24)

/* 主循环约 5 ms 一次；下面参数分别约为 20 ms、60 ms 和 10 ms。 */
#define TRACK_CORNER_ADVANCE_UPDATES   (4U)
#define TRACK_TURN_MIN_UPDATES         (12U)
#define TRACK_CENTER_CONFIRM_UPDATES   (2U)
#define TRACK_LOST_CONFIRM_UPDATES     (2U)
#define TRACK_CORNER_MIN_ACTIVE        (5U)

/* 两路电机安装方向；某侧前进方向相反时将对应值改为 -1。 */
#define LEFT_MOTOR_FORWARD_SIGN        (1)
#define RIGHT_MOTOR_FORWARD_SIGN       (1)

void Line_Tracking_Update(const uint16_t sensor_values[8]);

#endif /* LINE_TRACKING_H */
