#include "motor.h"

/** 初始化指定电机通道：退出TB6612待机、启动PWM，并将通道置为短刹车。 */
void motor_init(uint8_t motor_id)
{
    if ((motor_id != MOTOR_ID_A) && (motor_id != MOTOR_ID_B)) {
        return;
    }

    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    DL_Timer_startCounter(PWMAB_INST);
    motor_set_duty(motor_id, 0U);
    motor_set_direction(motor_id, MOTOR_DIRECTION_BRAKE);
}

/** 将PWM比较值限制在一个周期内，避免错误参数导致PWM异常。 */
static uint32_t limit_duty(uint32_t duty)
{
    return duty > MOTOR_PWM_PERIOD_COUNTS ? MOTOR_PWM_PERIOD_COUNTS : duty;
}

/** 设置指定通道的PWM占空比，duty范围为0~4000。 */
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    duty = limit_duty(duty);
    if (motor_id == MOTOR_ID_A) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    } else if (motor_id == MOTOR_ID_B) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

/** 设置指定通道方向：0短刹车、1正转、2反转。 */
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

/** 以-100~100的百分比驱动指定通道，正负号决定方向。 */
void motor_drive_percent(uint8_t motor_id, int16_t signed_percent)
{
    if (signed_percent > 100) {
        signed_percent = 100;
    } else if (signed_percent < -100) {
        signed_percent = -100;
    }

    if (signed_percent == 0) {
        motor_stop(motor_id);
        return;
    }

    const uint16_t magnitude = signed_percent < 0
        ? (uint16_t)(-signed_percent) : (uint16_t)signed_percent;
    motor_set_direction(motor_id, signed_percent > 0
        ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE);
    motor_set_duty(motor_id,
        ((uint32_t)magnitude * MOTOR_PWM_PERIOD_COUNTS) / 100U);
}

/** 停止指定通道：PWM清零并设置TB6612短刹车。 */
void motor_stop(uint8_t motor_id)
{
    motor_set_duty(motor_id, 0U);
    motor_set_direction(motor_id, MOTOR_DIRECTION_BRAKE);
}
