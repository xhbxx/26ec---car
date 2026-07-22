#include "line_tracking.h"

#include "motor.h"

typedef enum {
    TRACK_STATE_FOLLOW = 0,
    TRACK_STATE_CORNER_ADVANCE,
    TRACK_STATE_CORNER_TURN
} TrackState;

/**
 * 在方框黑线上完成正常差速循迹和固定方向的 90 度转弯。
 * 函数采用非阻塞状态机，每次主循环只更新一次动作，不使用延时等待。
 */
void Line_Tracking_Update(const uint16_t sensor_values[8])
{
    static const int8_t weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    static TrackState state = TRACK_STATE_FOLLOW;
    static uint8_t state_updates = 0U;
    static uint8_t center_updates = 0U;
    static uint8_t lost_updates = 0U;
    uint8_t active[8];
    uint8_t active_count = 0U;
    int16_t weighted_sum = 0;
    uint8_t channel;
    uint8_t corner_detected;

    if (sensor_values == 0) {
        return;
    }

    for (channel = 0U; channel < 8U; channel++) {
        active[channel] =
            (sensor_values[channel] == LINE_ACTIVE_LEVEL) ? 1U : 0U;
        if (active[channel] != 0U) {
            active_count++;
            weighted_sum += weights[channel];
        }
    }

    if (active_count == 0U) {
        if (lost_updates < TRACK_LOST_CONFIRM_UPDATES) {
            lost_updates++;
        }
    } else {
        lost_updates = 0U;
    }

    /*
     * 直角拐点通常表现为多路同时压线，或方框转向一侧的最外探头压线。
     * 方框四个角使用同一转向方向，避免在宽黑线处无法判断左右。
     */
    corner_detected = (active_count >= TRACK_CORNER_MIN_ACTIVE) ? 1U : 0U;
    if (TRACK_SQUARE_TURN_DIRECTION > 0) {
        if (((active[6] != 0U) || (active[7] != 0U)) &&
            (active_count >= 2U)) {
            corner_detected = 1U;
        }
    } else {
        if (((active[0] != 0U) || (active[1] != 0U)) &&
            (active_count >= 2U)) {
            corner_detected = 1U;
        }
    }

    if (state == TRACK_STATE_FOLLOW) {
        int16_t error;
        int16_t correction;
        int16_t left_speed;
        int16_t right_speed;

        if (corner_detected != 0U) {
            /* 先向前越过方框拐点，使车体旋转中心接近直角顶点。 */
            state = TRACK_STATE_CORNER_ADVANCE;
            state_updates = 0U;
            motor_drive_percent(MOTOR_ID_A,
                TRACK_BASE_SPEED_PERCENT * LEFT_MOTOR_FORWARD_SIGN);
            motor_drive_percent(MOTOR_ID_B,
                TRACK_BASE_SPEED_PERCENT * RIGHT_MOTOR_FORWARD_SIGN);
            return;
        }

        if (lost_updates >= TRACK_LOST_CONFIRM_UPDATES) {
            /* 已越过尖角且前方无黑线时，直接开始方框固定方向转弯。 */
            state = TRACK_STATE_CORNER_TURN;
            state_updates = 0U;
            center_updates = 0U;
            return;
        }

        if (active_count == 0U) {
            /* 单帧丢线先保持直行，过滤灰度模块瞬时抖动。 */
            motor_drive_percent(MOTOR_ID_A,
                TRACK_BASE_SPEED_PERCENT * LEFT_MOTOR_FORWARD_SIGN);
            motor_drive_percent(MOTOR_ID_B,
                TRACK_BASE_SPEED_PERCENT * RIGHT_MOTOR_FORWARD_SIGN);
            return;
        }

        error = (int16_t)(weighted_sum / (int16_t)active_count);
        correction = (int16_t)((error * 3) / 4);
        if (correction > TRACK_MAX_CORRECTION) {
            correction = TRACK_MAX_CORRECTION;
        } else if (correction < -TRACK_MAX_CORRECTION) {
            correction = -TRACK_MAX_CORRECTION;
        }

        left_speed = (int16_t)(TRACK_BASE_SPEED_PERCENT + correction);
        right_speed = (int16_t)(TRACK_BASE_SPEED_PERCENT - correction);
        motor_drive_percent(MOTOR_ID_A,
            (int16_t)(left_speed * LEFT_MOTOR_FORWARD_SIGN));
        motor_drive_percent(MOTOR_ID_B,
            (int16_t)(right_speed * RIGHT_MOTOR_FORWARD_SIGN));
        return;
    }

    if (state == TRACK_STATE_CORNER_ADVANCE) {
        motor_drive_percent(MOTOR_ID_A,
            TRACK_BASE_SPEED_PERCENT * LEFT_MOTOR_FORWARD_SIGN);
        motor_drive_percent(MOTOR_ID_B,
            TRACK_BASE_SPEED_PERCENT * RIGHT_MOTOR_FORWARD_SIGN);
        state_updates++;
        if (state_updates >= TRACK_CORNER_ADVANCE_UPDATES) {
            state = TRACK_STATE_CORNER_TURN;
            state_updates = 0U;
            center_updates = 0U;
        }
        return;
    }

    /* 左右轮反向，原地完成接近 90 度的固定方向转弯。 */
    if (TRACK_SQUARE_TURN_DIRECTION > 0) {
        motor_drive_percent(MOTOR_ID_A,
            TRACK_CORNER_TURN_SPEED * LEFT_MOTOR_FORWARD_SIGN);
        motor_drive_percent(MOTOR_ID_B,
            -TRACK_CORNER_TURN_SPEED * RIGHT_MOTOR_FORWARD_SIGN);
    } else {
        motor_drive_percent(MOTOR_ID_A,
            -TRACK_CORNER_TURN_SPEED * LEFT_MOTOR_FORWARD_SIGN);
        motor_drive_percent(MOTOR_ID_B,
            TRACK_CORNER_TURN_SPEED * RIGHT_MOTOR_FORWARD_SIGN);
    }
    state_updates++;

    /* 离开原来的宽线后，中间两路连续检测到新边才恢复正常循迹。 */
    if ((state_updates >= TRACK_TURN_MIN_UPDATES) &&
        ((active[3] != 0U) || (active[4] != 0U)) &&
        (active_count <= 3U)) {
        center_updates++;
    } else {
        center_updates = 0U;
    }
    if (center_updates >= TRACK_CENTER_CONFIRM_UPDATES) {
        state = TRACK_STATE_FOLLOW;
        state_updates = 0U;
        center_updates = 0U;
        lost_updates = 0U;
    }
}
