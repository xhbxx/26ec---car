#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include <stdint.h>

/* 当前灰度模块检测到黑线时返回 1。 */
#define LINE_ACTIVE_LEVEL                (1U)
#define TRACK_AUTO_ACTIVE_LEVEL          (0U)

/* 正常循迹速度和差速修正上限，单位为 motor_drive_percent() 百分比。 */
#define TRACK_REFERENCE_BASE_PERCENT     (30)// 中间直行基础速度
#define TRACK_SLOWDOWN_PER_SENSOR         (5)// 每偏一格降低多少速度
#define TRACK_MIN_FORWARD_PERCENT        (5)// 转弯时最低前进速度
#define TRACK_MAX_CORRECTION_PERCENT      (20)// 最大左右差速

/* 0~7 编号中，只有 3、4 两个中央探头同时且单独有效时允许直行。 */
#define TRACK_CENTER_MASK                ((1U << 3) | (1U << 4))
#define TRACK_CENTER_ESCAPE_PERCENT      (10)

#define TRACK_LOST_LEFT_PERCENT        (15)
#define TRACK_LOST_RIGHT_PERCENT       (15)
#define TRACK_LOST_CENTER_PERCENT      (15)

/* 方框转弯方向：1 为连续右转（顺时针），-1 为连续左转（逆时针）。 */

/* 主循环约 5 ms 一次；下面参数分别约为 20 ms、60 ms 和 10 ms。 */

/* 两路电机安装方向；某侧前进方向相反时将对应值改为 -1。 */
#define LEFT_MOTOR_FORWARD_SIGN        (1)
#define RIGHT_MOTOR_FORWARD_SIGN       (1)

void Line_Tracking_Update(const uint16_t sensor_values[8]);

#endif /* LINE_TRACKING_H */
