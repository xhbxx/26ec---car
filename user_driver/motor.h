#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

/* 与empty.syscfg中的PWM周期一致。 */
#define MOTOR_PWM_PERIOD_COUNTS    (4000U)

#define MOTOR_ID_A                 (1U)
#define MOTOR_ID_B                 (2U)

#define MOTOR_DIRECTION_BRAKE      (0U)
#define MOTOR_DIRECTION_FORWARD    (1U)
#define MOTOR_DIRECTION_REVERSE    (2U)

/*
 * 编码器安装在电机轴，只读取一路 A 相上升沿：
 * 输出轴转一圈计数 = 13线 × 28减速比 = 364。
 * 如果以后改成 A/B 两相四倍频，此处应再乘以 4。
 */
#define MOTOR_ENCODER_LINES              (13L)
#define MOTOR_GEAR_RATIO                 (28L)
#define MOTOR_ENCODER_COUNT_MULTIPLIER   (1L)
#define MOTOR_COUNTS_PER_OUTPUT_REV      \
    (MOTOR_ENCODER_LINES * MOTOR_GEAR_RATIO * MOTOR_ENCODER_COUNT_MULTIPLIER)
#define MOTOR_SPEED_SAMPLE_MS            (50L)

/* 100% 循迹速度对应的减速箱输出轴转速，可按电机实测最高转速修改。 */
#define MOTOR_MAX_OUTPUT_RPM             (125L)
#define MOTOR_RPM_SCALE                  (10L) /* 内部以0.1 rpm计算。 */
#define MOTOR_WHEEL_DIAMETER_MM          (65L)
#define MOTOR_PI_X10000                  (31416L)
#define MOTOR_MAX_OUTPUT_SPEED_MMPS      \
    ((MOTOR_MAX_OUTPUT_RPM * MOTOR_PI_X10000 * MOTOR_WHEEL_DIAMETER_MM) \
        / (60L * 10000L))

/* PID增益放大100倍，单位为“PWM百分比/RPM”。40表示0.40。 */
#define MOTOR_PID_GAIN_SCALE         (100L)
#define MOTOR_PID_KP                 (40)
#define MOTOR_PID_KI                 (20)
#define MOTOR_PID_KD                 (0)

/* PID may add this much PWM above the requested speed, but cannot reach 100% from a 40% target. */
#define MOTOR_PID_DUTY_HEADROOM_PERCENT (15)

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);
void motor_drive_percent(uint8_t motor_id, int16_t signed_percent);
void motor_drive_rpm(uint8_t motor_id, int16_t signed_rpm);
void motor_drive_mmps(uint8_t motor_id, int16_t signed_mmps);
int16_t motor_get_target_percent(uint8_t motor_id);
int32_t motor_get_target_speed_percent(uint8_t motor_id);
int32_t motor_get_actual_speed_percent(uint8_t motor_id);
int32_t motor_get_target_rpm_x10(uint8_t motor_id);
int32_t motor_get_actual_rpm_x10(uint8_t motor_id);
int32_t motor_get_target_mmps_x10(uint8_t motor_id);
int32_t motor_get_actual_mmps_x10(uint8_t motor_id);
int32_t motor_get_pid_kp(void);
int32_t motor_get_pid_ki(void);
int32_t motor_get_pid_kd(void);
void motor_adjust_pid_kp(int32_t delta);
void motor_adjust_pid_ki(int32_t delta);
void motor_stop(uint8_t motor_id);

#endif /* MOTOR_H */
