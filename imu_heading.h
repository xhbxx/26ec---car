#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <stdint.h>

void ImuHeading_Init(uint32_t now_ms);
void ImuHeading_Update(uint32_t now_ms);
void ImuHeading_ResetAngle(void);
uint8_t ImuHeading_IsReady(void);
uint8_t ImuHeading_IsOnline(void);
int32_t ImuHeading_GetAngleMdeg(void);
int32_t ImuHeading_GetRateMdps(void);
int16_t ImuHeading_GetBiasRaw(void);
int16_t ImuHeading_GetAccelXMg(void);
int16_t ImuHeading_GetAccelYMg(void);
int16_t ImuHeading_GetAccelZMg(void);

#endif /* IMU_HEADING_H */
