#include "chassis_controller.h"

#include "car_config.h"
#include "encoder.h"
#include "imu_heading.h"
#include "motor.h"

static ChassisState g_state = CHASSIS_WAIT;
static uint8_t g_task_mode = 2U;
static uint32_t g_start_ms;
static uint32_t g_finish_ms;
static uint32_t g_last_update_ms;
static int32_t g_left_start_count;
static int32_t g_right_start_count;
static int32_t g_state_start_distance_mm;
static int32_t g_center_command_x1000;
static int32_t g_steering_command_x1000;
static int32_t g_left_command_x1000;
static int32_t g_right_command_x1000;
static ChassisState g_stop_final_state;
static uint8_t g_marker_samples;

static int32_t chassis_abs32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t chassis_limit32(int32_t value, int32_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static int32_t chassis_accel_mmps2(void)
{
    if (g_task_mode == 4U) {
        return TASK4_ACCEL_MMPS2;
    }
    return TASK5_ACCEL_MMPS2;
}

static int32_t chassis_decel_mmps2(void)
{
    if (g_task_mode == 4U) {
        return TASK4_DECEL_MMPS2;
    }
    return TASK5_DECEL_MMPS2;
}

static int32_t chassis_total_distance_mm(void)
{
    int32_t left = Encoder_Get_Total_Count(0U) - g_left_start_count;
    int32_t right = Encoder_Get_Total_Count(1U) - g_right_start_count;
    int32_t average = (left + right) / 2;

    return (int32_t)(((int64_t)average * MOTOR_PI_X10000 *
        MOTOR_WHEEL_DIAMETER_MM) /
        ((int64_t)MOTOR_COUNTS_PER_OUTPUT_REV * 10000LL));
}

static int32_t chassis_task2_braking_distance_mm(void)
{
    int32_t left_mmps = chassis_abs32(
        motor_get_actual_mmps_x10(MOTOR_ID_A)) / 10L;
    int32_t right_mmps = chassis_abs32(
        motor_get_actual_mmps_x10(MOTOR_ID_B)) / 10L;
    int32_t speed_mmps = (left_mmps + right_mmps) / 2L;

    return (int32_t)(((int64_t)speed_mmps * speed_mmps) /
        (2LL * TASK2_ACTIVE_BRAKE_DECEL_MMPS2)) +
        TASK2_BRAKE_MARGIN_MM;
}

static int32_t chassis_slew_speed(int32_t current_x1000,
    int32_t target_mmps, uint32_t elapsed_ms)
{
    int32_t target_x1000 = target_mmps * 1000L;
    int32_t current_abs = chassis_abs32(current_x1000);
    int32_t target_abs = chassis_abs32(target_x1000);
    int32_t rate = target_abs > current_abs
        ? chassis_accel_mmps2() : chassis_decel_mmps2();
    int32_t step = (int32_t)((uint64_t)rate * elapsed_ms);

    if (target_x1000 > current_x1000) {
        if (target_x1000 - current_x1000 <= step) {
            return target_x1000;
        }
        return current_x1000 + step;
    }
    if (current_x1000 - target_x1000 <= step) {
        return target_x1000;
    }
    return current_x1000 - step;
}

static int32_t chassis_slew_steering(int32_t current_x1000,
    int32_t target_x1000, uint32_t elapsed_ms)
{
    int32_t step = (int32_t)((uint64_t)LINE_STEERING_SLEW_MMPS2 *
        elapsed_ms);

    if (target_x1000 > current_x1000) {
        if (target_x1000 - current_x1000 <= step) {
            return target_x1000;
        }
        return current_x1000 + step;
    }
    if (current_x1000 - target_x1000 <= step) {
        return target_x1000;
    }
    return current_x1000 - step;
}

static void chassis_apply_wheels(int32_t left_mmps, int32_t right_mmps,
    uint32_t elapsed_ms)
{
    int32_t speed_limit = MOTOR_MAX_OUTPUT_SPEED_MMPS;
    int32_t center_target_x1000;
    int32_t steering_target_x1000;
    int32_t steering_limit_x1000;

    left_mmps = chassis_limit32(left_mmps, speed_limit);
    right_mmps = chassis_limit32(right_mmps, speed_limit);

    center_target_x1000 = ((left_mmps + right_mmps) * 1000L) / 2L;
    steering_target_x1000 = ((left_mmps - right_mmps) * 1000L) / 2L;

    /* Mode 2 直接设置目标速度；Mode 4/5 保留平滑加减速。 */
    if (g_task_mode == 2U) {
        g_center_command_x1000 = center_target_x1000;
    } else {
        g_center_command_x1000 = chassis_slew_speed(
            g_center_command_x1000, center_target_x1000 / 1000L,
            elapsed_ms);
    }

    /* During startup, steering may stop one wheel but cannot reverse it. */
    steering_limit_x1000 = chassis_abs32(g_center_command_x1000);
    steering_target_x1000 = chassis_limit32(
        steering_target_x1000, steering_limit_x1000);
    g_steering_command_x1000 = chassis_slew_steering(
        g_steering_command_x1000, steering_target_x1000, elapsed_ms);
    g_steering_command_x1000 = chassis_limit32(
        g_steering_command_x1000, steering_limit_x1000);

    g_left_command_x1000 = g_center_command_x1000 +
        g_steering_command_x1000;
    g_right_command_x1000 = g_center_command_x1000 -
        g_steering_command_x1000;

    motor_drive_mmps(MOTOR_ID_A,
        (int16_t)((g_left_command_x1000 / 1000L) *
            CHASSIS_LEFT_FORWARD_SIGN));
    motor_drive_mmps(MOTOR_ID_B,
        (int16_t)((g_right_command_x1000 / 1000L) *
            CHASSIS_RIGHT_FORWARD_SIGN));
}

static void chassis_brake(void)
{
    motor_stop(MOTOR_ID_A);
    motor_stop(MOTOR_ID_B);
    g_center_command_x1000 = 0;
    g_steering_command_x1000 = 0;
    g_left_command_x1000 = 0;
    g_right_command_x1000 = 0;
}

static int32_t chassis_line_correction(const LineSensorData *line)
{
    /* Infrared is reserved for the finish marker, not continuous steering. */
    (void)line;
    return 0;
}

static void chassis_set_state(ChassisState state)
{
    g_state = state;
    g_state_start_distance_mm = chassis_total_distance_mm();
}

static int32_t chassis_straight_speed(void)
{
    if (g_task_mode == 2U) {
        return TASK2_STRAIGHT_MMPS;
    }
    if (g_task_mode == 4U) {
        return TASK4_STRAIGHT_MMPS;
    }
    return TASK5_STRAIGHT_MMPS;
}

static int32_t chassis_curve_speed(void)
{
    if (g_task_mode == 2U) {
        return TASK2_CURVE_MMPS;
    }
    return TASK5_CURVE_MMPS;
}

static int32_t chassis_cd_straight_distance_mm(void)
{
    return (g_task_mode == 2U)
        ? TASK2_CD_STRAIGHT_MM : TASK5_CD_STRAIGHT_MM;
}

static int32_t chassis_lap_length_mm(void)
{
    return (g_task_mode == 2U)
        ? TASK2_LAP_LENGTH_MM : TASK5_LAP_LENGTH_MM;
}

static void chassis_drive_straight(int32_t base_mmps,
    const LineSensorData *line, uint32_t elapsed_ms)
{
    int32_t line_correction;

    if (ImuHeading_IsReady() != 0U) {
        int32_t target_angle_mdeg = 0L;
        int32_t gain;
        int32_t limit;

        if (g_state == CHASSIS_STRAIGHT_CD) {
            target_angle_mdeg = CURVE_TARGET_ANGLE_MDEG;
        } else if (g_state == CHASSIS_FINAL_APPROACH) {
            target_angle_mdeg = 2L * CURVE_TARGET_ANGLE_MDEG;
        }
        if (g_task_mode == 2U) {
            gain = TASK2_GYRO_STRAIGHT_KP_MMPS_PER_DEG;
            limit = TASK2_GYRO_STRAIGHT_LIMIT_MMPS;
        } else {
            gain = TASK45_GYRO_STRAIGHT_KP_MMPS_PER_DEG;
            limit = TASK45_GYRO_STRAIGHT_LIMIT_MMPS;
        }

        /* 按当前直线路段的目标航向计算陀螺仪修正。 */
        int32_t heading_error = target_angle_mdeg -
            ImuHeading_GetAngleMdeg();

        line_correction = chassis_limit32(
            (int32_t)(((int64_t)heading_error * gain) / 1000L), limit);
    } else {
        line_correction = chassis_line_correction(line);
    }

    /* Mode 2 出弯后立即建立回正差速，不经过转向缓变。 */
    if ((g_task_mode == 2U) && (g_state == CHASSIS_STRAIGHT_CD)) {
        g_steering_command_x1000 = line_correction * 1000L;
    }

    chassis_apply_wheels(base_mmps + line_correction,
        base_mmps - line_correction, elapsed_ms);
}

static void chassis_drive_curve(uint32_t elapsed_ms,
    const LineSensorData *line, int32_t state_distance_mm)
{
    int32_t base_mmps = chassis_curve_speed();
    int32_t line_correction;

    if ((g_task_mode == 2U) || (g_task_mode == 5U)) {
        int32_t curve_start_angle_mdeg;
        int32_t curve_progress_angle_mdeg;
        int32_t target_angle_mdeg;
        int32_t measured_angle_mdeg;
        int32_t base_steering_mmps;
        int32_t gyro_gain;
        int32_t gyro_limit;
        int32_t gyro_correction = 0;

        curve_start_angle_mdeg = (g_state == CHASSIS_CURVE_DA)
            ? CURVE_TARGET_ANGLE_MDEG : 0L;
        curve_progress_angle_mdeg = (int32_t)(((int64_t)state_distance_mm *
            CURVE_TARGET_ANGLE_MDEG) / TRACK_HALF_CURVE_MM);
        curve_progress_angle_mdeg = chassis_limit32(
            curve_progress_angle_mdeg,
            CURVE_TARGET_ANGLE_MDEG);
        target_angle_mdeg = curve_start_angle_mdeg +
            curve_progress_angle_mdeg;
        measured_angle_mdeg = ImuHeading_GetAngleMdeg();
        if (g_task_mode == 2U) {
            base_steering_mmps = TASK2_CURVE_STEERING_MMPS;
            gyro_gain = TASK2_GYRO_CURVE_KP_MMPS_PER_DEG;
            gyro_limit = TASK2_GYRO_CURVE_LIMIT_MMPS;
        } else {
            base_steering_mmps = TASK5_CURVE_STEERING_MMPS;
            gyro_gain = TASK5_GYRO_CURVE_KP_MMPS_PER_DEG;
            gyro_limit = TASK5_GYRO_CURVE_LIMIT_MMPS;
        }
        if (ImuHeading_IsReady() != 0U) {
            gyro_correction = chassis_limit32(
                ((target_angle_mdeg - measured_angle_mdeg) / 1000L) *
                    gyro_gain, gyro_limit);
        }
        chassis_apply_wheels(
            base_mmps + base_steering_mmps + gyro_correction,
            base_mmps - base_steering_mmps - gyro_correction,
            elapsed_ms);
        return;
    }
    line_correction = chassis_line_correction(line);

    chassis_apply_wheels(
        base_mmps + line_correction,
        base_mmps - line_correction,
        elapsed_ms);
}

static void chassis_begin_stop(ChassisState final_state)
{
    g_stop_final_state = final_state;
    g_state = CHASSIS_STOPPING;
}

void ChassisController_Init(void)
{
    g_state = CHASSIS_WAIT;
    g_task_mode = 2U;
    g_start_ms = 0U;
    g_finish_ms = 0U;
    g_last_update_ms = 0U;
    g_left_start_count = 0;
    g_right_start_count = 0;
    g_state_start_distance_mm = 0;
    g_center_command_x1000 = 0;
    g_steering_command_x1000 = 0;
    g_left_command_x1000 = 0;
    g_right_command_x1000 = 0;
    g_stop_final_state = CHASSIS_COMPLETE;
    g_marker_samples = 0U;
    chassis_brake();
}

uint8_t ChassisController_Start(uint8_t task_mode, uint32_t now_ms)
{
    if ((task_mode != 2U) && (task_mode != 4U) &&
        (task_mode != 5U)) {
        return 0U;
    }
    g_task_mode = task_mode;
    g_start_ms = now_ms;
    g_finish_ms = 0U;
    g_last_update_ms = now_ms;
    g_left_start_count = Encoder_Get_Total_Count(0U);
    g_right_start_count = Encoder_Get_Total_Count(1U);
    g_state_start_distance_mm = 0;
    g_marker_samples = 0U;
    ImuHeading_ResetAngle();
    g_center_command_x1000 = 0;
    g_steering_command_x1000 = 0;
    g_left_command_x1000 = 0;
    g_right_command_x1000 = 0;
    g_stop_final_state = CHASSIS_COMPLETE;
    g_state = CHASSIS_STRAIGHT_AB;
    return 1U;
}

void ChassisController_Stop(uint32_t now_ms)
{
    (void)now_ms;
    chassis_begin_stop(CHASSIS_COMPLETE);
}

void ChassisController_Update(uint32_t now_ms, const LineSensorData *line)
{
    uint32_t elapsed_update = now_ms - g_last_update_ms;
    uint32_t mission_elapsed = now_ms - g_start_ms;
    int32_t total_distance;
    int32_t state_distance;
    uint32_t timeout;

    if ((g_state == CHASSIS_WAIT) || (g_state == CHASSIS_COMPLETE) ||
        (g_state == CHASSIS_ERROR)) {
        return;
    }
    if (elapsed_update == 0U) {
        return;
    }
    g_last_update_ms = now_ms;

    if (g_task_mode == 2U) {
        timeout = TASK2_TIMEOUT_MS;
    } else if (g_task_mode == 4U) {
        timeout = TASK4_TIMEOUT_MS;
    } else {
        timeout = TASK5_TIMEOUT_MS;
    }
    if ((mission_elapsed > timeout) && (g_state != CHASSIS_STOPPING)) {
        chassis_begin_stop(CHASSIS_ERROR);
        return;
    }

    total_distance = chassis_total_distance_mm();
    state_distance = total_distance - g_state_start_distance_mm;
    switch (g_state) {
    case CHASSIS_STRAIGHT_AB:
        chassis_drive_straight(chassis_straight_speed(), line,
            elapsed_update);
        if (state_distance >= TRACK_STRAIGHT_MM) {
            if (g_task_mode == 4U) {
                chassis_begin_stop(CHASSIS_COMPLETE);
            } else {
                chassis_set_state(CHASSIS_CURVE_BC);
            }
        }
        break;

    case CHASSIS_CURVE_BC:
        chassis_drive_curve(elapsed_update, line, state_distance);
        /* Mode 2 到达 180 度立即退出弯道，编码器距离作为陀螺仪异常保险。 */
        if ((g_task_mode == 2U) &&
            (((ImuHeading_IsReady() != 0U) &&
              (ImuHeading_GetAngleMdeg() >= CURVE_TARGET_ANGLE_MDEG)) ||
             (state_distance >= TRACK_HALF_CURVE_MM))) {
            /* 清除转向斜坡余量，下一周期立即按直线航向闭环。 */
            g_steering_command_x1000 = 0;
            chassis_set_state(CHASSIS_STRAIGHT_CD);
        } else if ((g_task_mode == 5U) &&
            (state_distance >= TRACK_HALF_CURVE_MM)) {
            chassis_set_state(CHASSIS_STRAIGHT_CD);
        }
        break;

    case CHASSIS_STRAIGHT_CD:
        chassis_drive_straight(chassis_straight_speed(), line,
            elapsed_update);
        if (state_distance >= chassis_cd_straight_distance_mm()) {
            chassis_set_state(CHASSIS_CURVE_DA);
        }
        break;

    case CHASSIS_CURVE_DA:
        chassis_drive_curve(elapsed_update, line, state_distance);
        if ((g_task_mode == 2U) &&
            (total_distance + chassis_task2_braking_distance_mm() >=
                chassis_lap_length_mm())) {
            chassis_begin_stop(CHASSIS_COMPLETE);
            break;
        }
        if ((g_task_mode == 5U) &&
            (total_distance >= chassis_lap_length_mm() - 220L) &&
            (line != 0) && (line->all_active != 0U)) {
            if (++g_marker_samples >= LINE_MARKER_STABLE_SAMPLES) {
                chassis_begin_stop(CHASSIS_COMPLETE);
                break;
            }
        } else {
            g_marker_samples = 0U;
        }
        if ((g_task_mode == 5U) &&
            (state_distance >= TRACK_HALF_CURVE_MM)) {
            chassis_set_state(CHASSIS_FINAL_APPROACH);
        }
        break;

    case CHASSIS_FINAL_APPROACH:
        chassis_drive_straight(FINAL_APPROACH_MMPS, line, elapsed_update);
        if ((line != 0) && (line->all_active != 0U)) {
            if (++g_marker_samples >= LINE_MARKER_STABLE_SAMPLES) {
                chassis_begin_stop(CHASSIS_COMPLETE);
                break;
            }
        } else {
            g_marker_samples = 0U;
        }
        if (total_distance >= chassis_lap_length_mm() +
            LAP_FALLBACK_EXTRA_MM) {
            chassis_begin_stop(CHASSIS_COMPLETE);
        }
        break;

    case CHASSIS_STOPPING:
        if (g_task_mode == 2U) {
            chassis_brake();
            g_finish_ms = now_ms;
            g_state = g_stop_final_state;
            break;
        }
        chassis_apply_wheels(0, 0, elapsed_update);
        if ((g_left_command_x1000 == 0) &&
            (g_right_command_x1000 == 0)) {
            chassis_brake();
            g_finish_ms = now_ms;
            g_state = g_stop_final_state;
        }
        break;

    default:
        chassis_brake();
        g_finish_ms = now_ms;
        g_state = CHASSIS_ERROR;
        break;
    }
}

ChassisState ChassisController_GetState(void)
{
    return g_state;
}

uint8_t ChassisController_GetTaskMode(void)
{
    return g_task_mode;
}

uint8_t ChassisController_IsRunning(void)
{
    return (g_state >= CHASSIS_STRAIGHT_AB) &&
        (g_state <= CHASSIS_STOPPING);
}

uint32_t ChassisController_GetElapsedMs(uint32_t now_ms)
{
    if (g_state == CHASSIS_WAIT) {
        return 0U;
    }
    if ((g_state == CHASSIS_COMPLETE) || (g_state == CHASSIS_ERROR)) {
        return g_finish_ms - g_start_ms;
    }
    return now_ms - g_start_ms;
}

int32_t ChassisController_GetDistanceMm(void)
{
    if (g_state == CHASSIS_WAIT) {
        return 0;
    }
    return chassis_total_distance_mm();
}

int32_t ChassisController_GetCurveAngleMdeg(void)
{
    if ((g_state != CHASSIS_CURVE_BC) &&
        (g_state != CHASSIS_CURVE_DA)) {
        return 0;
    }
    return ImuHeading_GetAngleMdeg();
}
