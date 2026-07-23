#include "motor.h"

#include "encoder.h"

typedef struct {
    int32_t integral;
    int32_t last_error;
    uint32_t duty;
} MotorPid;

/* 左右轮目标百分比和各自独立的 PID 状态。 */
static volatile int16_t motor_target_percent[2] = {0, 0};
static MotorPid motor_pid[2] = {{0, 0, 0U}, {0, 0, 0U}};

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
    motor_pid[index].integral = 0;
    motor_pid[index].last_error = 0;
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
    if (signed_percent != previous_target) {
        motor_pid[index].integral = 0;
        motor_pid[index].last_error = 0;
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
    motor_pid[index].integral = 0;
    motor_pid[index].last_error = 0;
    motor_pid[index].duty = 0U;
    motor_set_duty(motor_id, 0U);
    motor_set_direction(motor_id, MOTOR_DIRECTION_BRAKE);
}

/**
 * 根据目标百分比和实际脉冲计算一次 PID 输出。
 * 输出 = 百分比基础 PWM + P项 + I项 + D项。
 */
static uint32_t motor_pid_update(
    uint8_t index, int16_t target_percent, int32_t actual_pulses)
{
    const int32_t magnitude =
        target_percent < 0 ? -target_percent : target_percent;
    const int32_t target_pulses =
        (magnitude * MOTOR_ENCODER_PULSES_AT_100) / 100;
    const int32_t error = target_pulses - actual_pulses;
    const int32_t base_duty =
        (magnitude * MOTOR_PWM_PERIOD_COUNTS) / 100;
    int32_t output;

    motor_pid[index].integral += error;
    if (motor_pid[index].integral > MOTOR_PID_INTEGRAL_LIMIT) {
        motor_pid[index].integral = MOTOR_PID_INTEGRAL_LIMIT;
    } else if (motor_pid[index].integral < -MOTOR_PID_INTEGRAL_LIMIT) {
        motor_pid[index].integral = -MOTOR_PID_INTEGRAL_LIMIT;
    }

    output = base_duty
        + MOTOR_PID_KP * error
        + MOTOR_PID_KI * motor_pid[index].integral
        + MOTOR_PID_KD * (error - motor_pid[index].last_error);
    motor_pid[index].last_error = error;

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
