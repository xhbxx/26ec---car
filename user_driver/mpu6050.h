#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_RawData;

/* Initialize MPU6050 once after SYSCFG_DL_init(). Returns 1 when detected. */
uint8_t MPU6050_Init(void);

/* Called by the PB1 GPIO interrupt. It only records the data-ready event. */
void MPU6050_OnInterrupt(void);

/* Read a new sample after INT is asserted. Returns 1 when data was updated. */
uint8_t MPU6050_Update(void);

/* Copy the most recently received raw accelerometer and gyroscope values. */
void MPU6050_GetRawData(MPU6050_RawData *data);

/* Return 1 after WHO_AM_I and all configuration writes have succeeded. */
uint8_t MPU6050_IsOnline(void);

#endif
