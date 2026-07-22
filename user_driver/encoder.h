#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
void Encoder_Sample_ADC(void);
void Encoder_Record_Pulse(uint8_t motor_index);
int32_t Encoder_Get_Count(uint8_t motor_id);
int32_t Encoder_Get_Last_Count(uint8_t motor_id);
uint16_t Encoder_Get_ADC(uint8_t motor_id);

#endif /* ENCODER_H */
