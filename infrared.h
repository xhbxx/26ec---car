#ifndef INFRARED_H
#define INFRARED_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

/* Four direct digital infrared inputs, ordered from the leftmost to rightmost. */
#define IR1_PORT        GPIOB
#define IR1_PIN         DL_GPIO_PIN_14
#define IR2_PORT        GPIOB
#define IR2_PIN         DL_GPIO_PIN_15
#define IR3_PORT        GPIOB
#define IR3_PIN         DL_GPIO_PIN_16
#define IR4_PORT        GPIOA
#define IR4_PIN         DL_GPIO_PIN_14
void Infrared_Init(void);
uint8_t Infrared_Read(void);

#endif /* INFRARED_H */
