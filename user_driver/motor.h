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

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);
void motor_drive_percent(uint8_t motor_id, int16_t signed_percent);
void motor_stop(uint8_t motor_id);

#endif /* MOTOR_H */
