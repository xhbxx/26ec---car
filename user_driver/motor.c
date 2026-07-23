#include "motor.h"

#include "encoder.h"

typedef struct {
    int32_t last_error;
    int32_t actual_speed_mm_s;
    uint32_t duty;
} MotorPid;

/* 左右轮目标百分比和各自独立的 PID 状态。 */
static volatile int16_t motor_target_percent[2] = {0, 0};
static MotorPid motor_pid[2] = {{0, 0, 0U}, {0, 0, 0U}};
static volatile int32_t motor_pid_kp = MOTOR_PID_KP;
static volatile int32_t motor_pid_ki = MOTOR_PID_KI;

/** Return the runtime proportional gain used by both motor PID loops. */
int32_t motor_get_pid_kp(void)
{
    return motor_pid_kp;
}

/** Return the runtime integral gain used by both motor PID loops. */
int32_t motor_get_pid_ki(void)
{
    return motor_pid_ki;
}

/** Adjust Kp by one key step and keep it in a practical non-negative range. */
void motor_adjust_pid_kp(int32_t delta)
{
    int32_t value = motor_pid_kp + delta;
    if (value < 0) {
        value = 0;
    } else if (value > 100) {
        value = 100;
    }
    motor_pid_kp = value;
}

/** Adjust Ki by one key step for the incremental PI controller. */
void motor_adjust_pid_ki(int32_t delta)
{
    int32_t value = motor_pid_ki + delta;
    if (value < 0) {
        value = 0;
    } else if (value > 100) {
        value = 100;
    }
    motor_pid_ki = value;
}

/** 初始化指定电机通道；PWM 和 STBY 只在主函数启动阶段配置。 */
void motor_init(uint8_t motor_id)
{
    uint8_t index;
    if ((motor_id != MOTOR_ID_A) && (motor_id != MOTOR_ID_B)) {
        return;
    }
    index = (uint8_t)(motor_id - MOTOR_ID_A);
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    DL_Timer_startCounter(PWMAB_INST);
    motor_target_percent[index] = 0;
    motor_pid[index].last_error = 0;
    motor_pid[index].actual_speed_mm_s = 0;
    motor_pid[index].duty = 0U;
    motor_set_duty(motor_id, 0U);
    motor_set_direction(motor_id, MOTOR_DIRECTION_BRAKE);
}

/** 限制 PWM 比较值，防止 PID 调节超出周期范围。 */
static uint32_t limit_duty(uint32_t duty)
{
    return duty > MOTOR_PWM_PERIOD_COUNTS ? MOTOR_PWM_PERIOD_COUNTS : duty;
}

/** 设置指定通道的 PWM 占空比。 */
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    duty = limit_duty(duty);
    if (motor_id == MOTOR_ID_A) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    } else if (motor_id == MOTOR_ID_B) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

/** 设置指定通道方向：短刹车、正转或反转。 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == MOTOR_ID_A) {
        if (direction == MOTOR_DIRECTION_FORWARD) {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else if (direction == MOTOR_DIRECTION_REVERSE) {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    } else if (motor_id == MOTOR_ID_B) {
        if (direction == MOTOR_DIRECTION_FORWARD) {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else if (direction == MOTOR_DIRECTION_REVERSE) {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}

/**
 * 原有调速入口：百分比决定目标速度，正负号只决定方向。
 * 实际 PWM 由 50 ms PID 中断根据左右编码器脉冲自动修正。
 */
void motor_drive_percent(uint8_t motor_id, int16_t signed_percent)
{
    uint8_t index;
    int16_t magnitude;
    int16_t previous_target;
    if ((motor_id != MOTOR_ID_A) && (motor_id != MOTOR_ID_B)) {
        return;
    }
    index = (uint8_t)(motor_id - MOTOR_ID_A);
    if (signed_percent > 100) {
        signed_percent = 100;
    } else if (signed_percent < -100) {
        signed_percent = -100;
    }
    previous_target = motor_target_percent[index];
    motor_target_percent[index] = signed_percent;
    if (signed_percent == 0) {
        motor_stop(motor_id);
        return;
    }
    magnitude = signed_percent < 0 ? (int16_t)(-signed_percent) : signed_percent;
    motor_set_direction(motor_id, signed_percent > 0
        ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE);
    /* 目标变化时才更新基础 PWM，避免主循环反复覆盖 PID 输出。 */
    /* Keep PID state across normal tracking corrections; seed PWM only on start/reverse. */
    if ((previous_target == 0) ||
        ((previous_target < 0) != (signed_percent < 0))) {
        motor_pid[index].duty =
            ((uint32_t)magnitude * MOTOR_PWM_PERIOD_COUNTS) / 100U;
        motor_set_duty(motor_id, motor_pid[index].duty);
    }
}

/** 停止指定通道并清除该通道 PID 状态。 */
/* Return the current target percentage for OLED/debug display. */
int16_t motor_get_target_percent(uint8_t motor_id)
{
    if ((motor_id != MOTOR_ID_A) && (motor_id != MOTOR_ID_B)) {
        return 0;
    }
    return motor_target_percent[(uint8_t)(motor_id - MOTOR_ID_A)];
}

void motor_stop(uint8_t motor_id)
{
    uint8_t index;
    if ((motor_id != MOTOR_ID_A) && (motor_id != MOTOR_ID_B)) {
        return;
    }
    index = (uint8_t)(motor_id - MOTOR_ID_A);
    motor_target_percent[index] = 0;
    motor_pid[index].last_error = 0;
    motor_pid[index].actual_speed_mm_s = 0;
    motor_pid[index].duty = 0U;
    motor_set_duty(motor_id, 0U);
    motor_set_direction(motor_id, MOTOR_DIRECTION_BRAKE);
}

/**
 * 根据目标百分比和实际脉冲计算一次 PID 输出。
 * 输出 = 百分比基础 PWM + P项 + I项 + D项。
 */
/** Convert encoder pulses collected over one PID period to wheel speed in mm/s. */
static int32_t motor_calculate_speed_mm_s(int32_t pulses)
{
    /* speed = pulses / 260 * PI * 67 / 0.05; PI is scaled by 10000. */
    return (pulses * 31416L * MOTOR_WHEEL_DIAMETER_MM) /
        (10L * MOTOR_ENCODER_PULSES_PER_REV * MOTOR_PID_PERIOD_MS);
}

/**
 * Use the same incremental PI formula as the supplied one-motor example:
 * PWM += Kp * (current_error - last_error) + Ki * current_error.
 */
static uint32_t motor_pid_update(
    uint8_t index, int16_t target_percent, int32_t actual_pulses)
{
    const int32_t magnitude =
        target_percent < 0 ? -target_percent : target_percent;
    const int32_t target_speed_mm_s =
        (magnitude * MOTOR_MAX_TARGET_SPEED_MM_S) / 100L;
    const int32_t actual_speed_mm_s =
        motor_calculate_speed_mm_s(actual_pulses);
    const int32_t error = target_speed_mm_s - actual_speed_mm_s;
    const int32_t duty_increment =
        (motor_pid_kp * (error - motor_pid[index].last_error) +
         motor_pid_ki * error) / MOTOR_PID_GAIN_SCALE;
    int32_t output = (int32_t)motor_pid[index].duty + duty_increment;

    motor_pid[index].last_error = error;
    motor_pid[index].actual_speed_mm_s = actual_speed_mm_s;
    if (output < 0) {
        output = 0;
    } else if (output > MOTOR_PWM_PERIOD_COUNTS) {
        output = MOTOR_PWM_PERIOD_COUNTS;
    }
    motor_pid[index].duty = (uint32_t)output;
    return motor_pid[index].duty;
}

/** 50 ms 定时中断：读取左右脉冲，各调用一次相同的 PID 函数。 */
void MOTOR_PID_INST_IRQHandler(void)
{
    uint8_t index;
    if (DL_Timer_getPendingInterrupt(MOTOR_PID_INST) != DL_TIMER_IIDX_LOAD) {
        return;
    }
    for (index = 0U; index < 2U; index++) {
        const int16_t target = motor_target_percent[index];
        const int32_t actual_pulses = Encoder_Get_Count(index);

        if (target == 0) {
            continue;
        }
        motor_set_duty((uint8_t)(index + MOTOR_ID_A),
            motor_pid_update(index, target, actual_pulses));
    }
}
