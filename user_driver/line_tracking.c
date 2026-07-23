#include "line_tracking.h"

#include "grayscale_sensor.h"
#include "motor.h"

/* 0~7 路探头从左到右对应的位置权重。 */
static const int16_t tracking_weights[GRAYSCALE_SENSOR_CHANNELS] = {
    -7, -5, -3, -1, 1, 3, 5, 7
};

/** 对左右目标速度限幅后，直接送入现有两路电机调速函数。 */
static void tracking_apply_motor_targets(
    int16_t left_target, int16_t right_target)
{
    if (left_target > 100) {
        left_target = 100;
    } else if (left_target < -100) {
        left_target = -100;
    }
    if (right_target > 100) {
        right_target = 100;
    } else if (right_target < -100) {
        right_target = -100;
    }

    motor_drive_percent(MOTOR_ID_A,
        left_target * LEFT_MOTOR_FORWARD_SIGN);
    motor_drive_percent(MOTOR_ID_B,
        right_target * RIGHT_MOTOR_FORWARD_SIGN);
}

/**
 * 只使用八路黑线位置的加权平均计算循迹误差。
 * 黑线为 1；除了全部为 0 时直行，不再判断任何特定组合。
 */
void Line_Tracking_Update(
    const uint16_t sensor_values[GRAYSCALE_SENSOR_CHANNELS])
{
    uint8_t channel;
    uint8_t active_count = 0U;
    int32_t weight_sum = 0;
    int32_t error_x100;
    int32_t abs_error_x100;
    int16_t base_percent;
    int16_t correction_percent;

    if (sensor_values == 0) {
        return;
    }

    for (channel = 0U; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        if (sensor_values[channel] != 0U) {
            weight_sum += tracking_weights[channel];
            active_count++;
        }
    }

    /* 没有检测到黑线时，按基础速度继续直行。 */
    if (active_count == 0U) {
        tracking_apply_motor_targets(
            TRACK_REFERENCE_BASE_PERCENT, TRACK_REFERENCE_BASE_PERCENT);
        return;
    }

    /* 保留两位小数，避免多路同时检测时的整数除法跳变。 */
    error_x100 = (weight_sum * 100L) / active_count;
    abs_error_x100 = error_x100 < 0 ? -error_x100 : error_x100;

    /* 越靠近边缘，整体速度越低；最多降低 TRACK_EDGE_SLOWDOWN_PERCENT。 */
    base_percent = TRACK_REFERENCE_BASE_PERCENT - (int16_t)(
        (abs_error_x100 * TRACK_EDGE_SLOWDOWN_PERCENT) / 700L);
    if (base_percent < TRACK_MIN_FORWARD_PERCENT) {
        base_percent = TRACK_MIN_FORWARD_PERCENT;
    }

    /* 权重误差线性换算为左右轮差速，边缘权重对应最大修正量。 */
    correction_percent = (int16_t)(
        (error_x100 * TRACK_MAX_CORRECTION_PERCENT) / 700L);

    tracking_apply_motor_targets(
        base_percent - correction_percent,
        base_percent + correction_percent);
}
