#ifndef GRAYSCALE_SENSOR_H
#define GRAYSCALE_SENSOR_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

#define GRAYSCALE_SENSOR_CHANNELS (8U)

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t *sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif /* GRAYSCALE_SENSOR_H */
