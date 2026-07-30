#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float last_error;
    float derivative;

    float output;
    float max_output;
    float integral_limit;

    float dt;
    float derivative_alpha;
    uint8_t initialized;
} PID_TypeDef;

/** 初始化离散 PID 的增益、固定周期、积分限幅、输出限幅和微分滤波系数。 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
    float dt, float max_output, float integral_limit,
    float derivative_alpha);

/** 清除 PID 的积分、历史误差、滤波微分和输出。 */
void PID_Reset(PID_TypeDef *pid);

/** 按固定周期更新 PID，返回经过积分限幅、微分滤波和输出限幅的结果。 */
float PID_Update(PID_TypeDef *pid, float target, float feedback);

#endif /* PID_H */
