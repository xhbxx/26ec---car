#include "pid.h"

/** 将浮点数限制在指定的对称范围内。 */
static float PID_Limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
    float dt, float max_output, float integral_limit,
    float derivative_alpha)
{
    if (pid == 0) {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = (dt > 0.0f) ? dt : 0.001f;
    pid->max_output = (max_output >= 0.0f) ? max_output : -max_output;
    pid->integral_limit = (integral_limit >= 0.0f)
        ? integral_limit : -integral_limit;
    if (derivative_alpha < 0.0f) {
        derivative_alpha = 0.0f;
    } else if (derivative_alpha > 1.0f) {
        derivative_alpha = 1.0f;
    }
    pid->derivative_alpha = derivative_alpha;
    PID_Reset(pid);
}

void PID_Reset(PID_TypeDef *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0U;
}

float PID_Update(PID_TypeDef *pid, float target, float feedback)
{
    float error;
    float raw_derivative;

    if (pid == 0) {
        return 0.0f;
    }

    error = target - feedback;
    pid->integral += error * pid->dt;
    pid->integral = PID_Limit(pid->integral, pid->integral_limit);

    if (pid->initialized == 0U) {
        raw_derivative = 0.0f;
        pid->initialized = 1U;
    } else {
        raw_derivative = (error - pid->last_error) / pid->dt;
    }
    pid->derivative += pid->derivative_alpha *
        (raw_derivative - pid->derivative);

    pid->output = pid->Kp * error +
        pid->Ki * pid->integral +
        pid->Kd * pid->derivative;
    pid->output = PID_Limit(pid->output, pid->max_output);
    pid->last_error = error;
    return pid->output;
}
