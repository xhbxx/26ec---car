#include "line_tracking.h"

#include "motor.h"

/** 将速度限制在电机驱动接口允许的-100%～100%范围内。 */
static int16_t limit_motor_percent(int16_t speed)
{
    if (speed > 100) {
        return 100;
    }
    if (speed < -100) {
        return -100;
    }
    return speed;
}

/** 按左右轮实际安装方向输出速度，保证正的逻辑速度表示小车向前。 */
static void drive_chassis(int16_t left_speed, int16_t right_speed)
{
    motor_drive_percent(MOTOR_ID_A,
        limit_motor_percent((int16_t)(left_speed * LEFT_MOTOR_FORWARD_SIGN)));
    motor_drive_percent(MOTOR_ID_B,
        limit_motor_percent((int16_t)(right_speed * RIGHT_MOTOR_FORWARD_SIGN)));
}

/**
 * 根据八路灰度值进行差速循迹。
 * 0号到7号传感器应从车头左侧向右侧排列；检测不到线时按要求继续直行。
 */
void Line_Tracking_Update(const uint16_t sensor_values[8])
{
    static const int8_t weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    int16_t weighted_sum = 0;
    uint8_t active_count = 0U;
    uint8_t channel;
    int16_t error;
    int16_t correction;
    int16_t left_speed;
    int16_t right_speed;

    if (sensor_values == 0) {
        drive_chassis(TRACK_BASE_SPEED_PERCENT, TRACK_BASE_SPEED_PERCENT);
        return;
    }

    for (channel = 0U; channel < 8U; channel++) {
        if (sensor_values[channel] == LINE_ACTIVE_LEVEL) {
            weighted_sum += weights[channel];
            active_count++;
        }
    }

    if (active_count == 0U) {
        /* 八路都未检测到黑线：不找线、不停车，保持直线前行。 */
        drive_chassis(TRACK_BASE_SPEED_PERCENT, TRACK_BASE_SPEED_PERCENT);
        return;
    }

    error = (int16_t)(weighted_sum / (int16_t)active_count);
    correction = (int16_t)((error * 3) / 4);
    if (correction > TRACK_MAX_CORRECTION) {
        correction = TRACK_MAX_CORRECTION;
    } else if (correction < -TRACK_MAX_CORRECTION) {
        correction = -TRACK_MAX_CORRECTION;
    }

    /* 线在左侧时error<0：左轮减速、右轮加速；线在右侧时方向相反。 */
    left_speed = (int16_t)(TRACK_BASE_SPEED_PERCENT + correction);
    right_speed = (int16_t)(TRACK_BASE_SPEED_PERCENT - correction);
    drive_chassis(left_speed, right_speed);
}
