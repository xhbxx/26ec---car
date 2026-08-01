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

/**
 * 更新PID并允许调用者冻结积分；integral_enabled为0时不继续累计积分，
 * 适合目标死区、执行器未就绪和输出饱和期间使用。
 */
float PID_UpdateConditional(PID_TypeDef *pid, float target, float feedback,
    uint8_t integral_enabled);

/** 按0~1比例衰减已有积分，避免方向切换时保留过多旧方向积分。 */
void PID_DecayIntegral(PID_TypeDef *pid, float factor);

#endif /* PID_H */
