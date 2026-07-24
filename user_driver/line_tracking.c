#include "line_tracking.h"

#include "grayscale_sensor.h"
#include "mpu6050.h"
#include "motor.h"

/* 0~7 路探头从左到右对应的位置权重。 */
static const int16_t tracking_weights[GRAYSCALE_SENSOR_CHANNELS] = {
    -7, -5, -3, -1, 1, 3, 5, 7
};

static int32_t gyro_bias_sum = 0;
static uint16_t gyro_calibration_count = 0U;
static int16_t gyro_raw = 0;
static int16_t gyro_bias = 0;
static int16_t gyro_error = 0;
static int16_t gyro_correction = 0;
static uint8_t gyro_ready = 0U;

/** Update GY-6500 zero bias; return 1 only after stationary calibration. */
static uint8_t tracking_update_gyro(void)
{
    MPU6050_RawData data;
    int32_t corrected;

    if (MPU6050_GetStatus() != 1U) {
        gyro_error = 0;
        gyro_correction = 0;
        return gyro_ready;
    }

    MPU6050_GetRawData(&data);
    gyro_raw = data.gyro_z;
    if (gyro_ready == 0U) {
        gyro_bias_sum += gyro_raw;
        gyro_calibration_count++;
        if (gyro_calibration_count >= TRACK_GYRO_CALIBRATION_SAMPLES) {
            gyro_bias = (int16_t)(gyro_bias_sum /
                (int32_t)TRACK_GYRO_CALIBRATION_SAMPLES);
            gyro_ready = 1U;
        }
        gyro_error = 0;
        gyro_correction = 0;
        return gyro_ready;
    }

    corrected = (int32_t)gyro_raw - gyro_bias;
    if ((corrected <= TRACK_GYRO_DEADBAND_RAW) &&
        (corrected >= -TRACK_GYRO_DEADBAND_RAW)) {
        corrected = 0;
    }
    gyro_error = (int16_t)corrected;
    corrected = (corrected * TRACK_GYRO_CORRECTION_SIGN) /
        TRACK_GYRO_RAW_PER_PERCENT;
    if (corrected > TRACK_GYRO_MAX_CORRECTION_PERCENT) {
        corrected = TRACK_GYRO_MAX_CORRECTION_PERCENT;
    } else if (corrected < -TRACK_GYRO_MAX_CORRECTION_PERCENT) {
        corrected = -TRACK_GYRO_MAX_CORRECTION_PERCENT;
    }
    gyro_correction = (int16_t)corrected;
    return 1U;
}

int16_t Line_Tracking_GetGyroRaw(void)
{
    return gyro_raw;
}

int16_t Line_Tracking_GetGyroBias(void)
{
    return gyro_bias;
}

int16_t Line_Tracking_GetGyroError(void)
{
    return gyro_error;
}

int16_t Line_Tracking_GetGyroCorrection(void)
{
    return gyro_correction;
}

uint8_t Line_Tracking_IsGyroReady(void)
{
    return gyro_ready;
}

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
uint8_t Line_Tracking_Update(
    const uint16_t sensor_values[GRAYSCALE_SENSOR_CHANNELS])
{
    uint8_t channel;
    uint8_t active_count = 0U;
    int32_t weight_sum = 0;
    int32_t error_x100;
    int32_t abs_error_x100;
    int16_t base_percent;
    int16_t correction_percent;
    const uint8_t gyro_is_ready = tracking_update_gyro();

    for (channel = 0U; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        if (sensor_values[channel] != 0U) {
            weight_sum += tracking_weights[channel];
            active_count++;
        }
    }

    /* 传感器在线时先静止校准零偏，防止固定零偏令小车持续转圈。 */
    if ((MPU6050_IsOnline() != 0U) && (gyro_is_ready == 0U)) {
        motor_stop(MOTOR_ID_A);
        motor_stop(MOTOR_ID_B);
        return active_count;
    }

    /* 丢线时使用校正后的Z轴角速度抑制转动；离线则两轮同速。 */
    if (active_count == 0U) {
        tracking_apply_motor_targets(
            TRACK_REFERENCE_BASE_PERCENT + gyro_correction,
            TRACK_REFERENCE_BASE_PERCENT - gyro_correction);
        return active_count;
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
     return active_count;
}
