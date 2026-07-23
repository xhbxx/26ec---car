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

/* 100% 速度在一个 50 ms PID 周期内对应的实测脉冲数。 */
#define MOTOR_FULLSPEED_PULSES_PER_50MS (38L)

/* Gains use 100x fixed point: 50 means 0.50 and 40 means 0.40. */
#define MOTOR_PID_GAIN_SCALE         (100L)
#define MOTOR_PID_KP                 (40)
#define MOTOR_PID_KI                 (20)
#define MOTOR_PID_KD                 (0)

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);
void motor_drive_percent(uint8_t motor_id, int16_t signed_percent);
int16_t motor_get_target_percent(uint8_t motor_id);
int32_t motor_get_target_speed_percent(uint8_t motor_id);
int32_t motor_get_actual_speed_percent(uint8_t motor_id);
int32_t motor_get_pid_kp(void);
int32_t motor_get_pid_ki(void);
int32_t motor_get_pid_kd(void);
void motor_adjust_pid_kp(int32_t delta);
void motor_adjust_pid_ki(int32_t delta);
void motor_stop(uint8_t motor_id);

#endif /* MOTOR_H */
