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

/* Initialize GY-6500/MPU6500 after SYSCFG_DL_init(); MPU6050 is compatible. */
uint8_t MPU6050_Init(void);

/* Called by the PB1 GPIO interrupt. It only records the data-ready event. */
void MPU6050_OnInterrupt(void);

/* Read a new sample after INT is asserted. Returns 1 when data was updated. */
uint8_t MPU6050_Update(void);

/* Copy the most recently received raw accelerometer and gyroscope values. */
void MPU6050_GetRawData(MPU6050_RawData *data);

/* Return 1 after WHO_AM_I and all configuration writes have succeeded. */
uint8_t MPU6050_IsOnline(void);

/* State: 0=not detected, 1=data valid, 2=read failed, 3=config failed. */
uint8_t MPU6050_GetStatus(void);

/* GY-6500 returns 0x70 (112); MPU6050 returns 0x68 (104). */
uint8_t MPU6050_GetWhoAmI(void);

/* Return the detected 7-bit I2C address, normally 0x68 or 0x69. */
uint8_t MPU6050_GetAddress(void);

#endif
