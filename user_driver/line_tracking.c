#include "line_tracking.h"

#include "grayscale_sensor.h"
#include "motor.h"

static int16_t g_last_correction_sign = 1;

/** 限制目标速度后立即更新左右电机，不进行渐变等待。 */
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
 * 根据八路数字灰度值更新左右轮目标速度。
 * 当前模块检测到黑线为 1，速度根据本次检测结果直接跳变。
 */
void Line_Tracking_Update(const uint16_t sensor_values[GRAYSCALE_SENSOR_CHANNELS])
{
    uint8_t raw_one_count = 0U;
    uint8_t active_level;
    uint8_t active_count = 0U;
    uint8_t active_mask = 0U;
    uint8_t channel;
    int32_t position_sum = 0;
    int32_t position_x100;
    int32_t error_x100;
    int32_t abs_error_x100;
    int16_t base_percent;
    int16_t correction_percent;
    int16_t left_target;
    int16_t right_target;

    if (sensor_values == 0) {
        return;
    }

    for (channel = 0U; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        if (sensor_values[channel] != 0U) {
            raw_one_count++;
        }
    }

    /* 没有任何一路检测到黑线时，按上一次方向直接搜索黑线。 */
    if (raw_one_count == 0U) {
        if (g_last_correction_sign > 0) {
            tracking_apply_motor_targets(
                TRACK_LOST_RIGHT_PERCENT, -TRACK_LOST_LEFT_PERCENT);
        } else {
            tracking_apply_motor_targets(
                -TRACK_LOST_LEFT_PERCENT, TRACK_LOST_RIGHT_PERCENT);
        }
        return;
    }

#if TRACK_AUTO_ACTIVE_LEVEL
    if (raw_one_count < (GRAYSCALE_SENSOR_CHANNELS / 2U)) {
        active_level = 1U;
    } else if (raw_one_count > (GRAYSCALE_SENSOR_CHANNELS / 2U)) {
        active_level = 0U;
    } else {
        active_level = LINE_ACTIVE_LEVEL;
    }
#else
    active_level = LINE_ACTIVE_LEVEL;
#endif

    for (channel = 0U; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        if (sensor_values[channel] == active_level) {
            position_sum += (int32_t)(channel + 1U);
            active_count++;
            active_mask |= (uint8_t)(1U << channel);
        }
    }

    if (active_count == 0U) {
        return;
    }

    /* 只有索引 3、4 两个中央探头同时且单独有效，目标差速才为 0。 */
    if (active_mask == TRACK_CENTER_MASK) {
        tracking_apply_motor_targets(
            TRACK_REFERENCE_BASE_PERCENT, TRACK_REFERENCE_BASE_PERCENT);
        return;
    }

    /* 位置使用放大 100 倍的定点数：1号为100，中心4.5为450，8号为800。 */
    position_x100 = (position_sum * 100) / active_count;
    error_x100 = 450 - position_x100;
    abs_error_x100 = error_x100 < 0 ? -error_x100 : error_x100;

    /* 偏离越大，整体前进速度越低；结果直接送入电机，不做渐变。 */
    base_percent = TRACK_REFERENCE_BASE_PERCENT -
        (int16_t)((abs_error_x100 * TRACK_SLOWDOWN_PER_SENSOR) / 100);
    if (base_percent < TRACK_MIN_FORWARD_PERCENT) {
        base_percent = TRACK_MIN_FORWARD_PERCENT;
    }

    correction_percent = (int16_t)(
        (error_x100 * TRACK_MAX_CORRECTION_PERCENT) / 350);
    if (correction_percent > TRACK_MAX_CORRECTION_PERCENT) {
        correction_percent = TRACK_MAX_CORRECTION_PERCENT;
    } else if (correction_percent < -TRACK_MAX_CORRECTION_PERCENT) {
        correction_percent = -TRACK_MAX_CORRECTION_PERCENT;
    }

    if (correction_percent > 0) {
        g_last_correction_sign = 1;
    } else if (correction_percent < 0) {
        g_last_correction_sign = -1;
    } else {
        /* 非中央对称组合平均值可能为中心，仍按最近方向做小幅修正。 */
        correction_percent =
            TRACK_CENTER_ESCAPE_PERCENT * g_last_correction_sign;
    }

    left_target = base_percent + correction_percent;
    right_target = base_percent - correction_percent;
    tracking_apply_motor_targets(left_target, right_target);
}
