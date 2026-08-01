#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

typedef struct {
    uint16_t raw[4];
    uint8_t mask;
    uint8_t active_count;
    uint8_t all_active;
    uint8_t line_valid;
    int32_t error_x100;
} LineSensorData;

void LineSensor_Init(void);
void LineSensor_Read(LineSensorData *data);

#endif /* LINE_SENSOR_H */
