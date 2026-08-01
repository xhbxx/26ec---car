#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <stdint.h>

#include "line_sensor.h"

typedef enum {
    CHASSIS_WAIT = 0,
    CHASSIS_STRAIGHT_AB,
    CHASSIS_CURVE_BC,
    CHASSIS_STRAIGHT_CD,
    CHASSIS_CURVE_DA,
    CHASSIS_FINAL_APPROACH,
    CHASSIS_STOPPING,
    CHASSIS_COMPLETE,
    CHASSIS_ERROR
} ChassisState;

void ChassisController_Init(void);
uint8_t ChassisController_Start(uint8_t task_mode, uint32_t now_ms);
void ChassisController_Stop(uint32_t now_ms);
void ChassisController_Update(uint32_t now_ms, const LineSensorData *line);
ChassisState ChassisController_GetState(void);
uint8_t ChassisController_GetTaskMode(void);
uint8_t ChassisController_IsRunning(void);
uint32_t ChassisController_GetElapsedMs(uint32_t now_ms);
int32_t ChassisController_GetDistanceMm(void);
int32_t ChassisController_GetCurveAngleMdeg(void);

#endif /* CHASSIS_CONTROLLER_H */
