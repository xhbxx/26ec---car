#include "encoder.h"

/* 两路编码器只使用 A 相；每个上升沿计为一个脉冲。 */
static volatile int32_t encoder_count[2] = {0, 0};
static volatile int32_t encoder_last_count[2] = {0, 0};

/** 初始化左右编码器计数，主函数中只调用一次。 */
void Encoder_Init(void)
{
    encoder_count[0] = 0;
    encoder_count[1] = 0;
    encoder_last_count[0] = 0;
    encoder_last_count[1] = 0;
}

/**
 * 保留原采样函数以兼容旧调用。
 * 当前脉冲由 GPIO 上升沿中断记录，不再依赖主循环轮询。
 */
void Encoder_Sample_ADC(void)
{
}

/** 由共享 GPIO 中断入口分别记录左轮或右轮的一个脉冲。 */
void Encoder_Record_Pulse(uint8_t motor_index)
{
    if (motor_index < 2U) {
        encoder_count[motor_index]++;
    }
}

/** 取出一个 PID 周期内的脉冲数，并清零下一周期计数。 */
int32_t Encoder_Get_Count(uint8_t motor_id)
{
    int32_t count;
    if (motor_id >= 2U) {
        return 0;
    }
    count = encoder_count[motor_id];
    encoder_last_count[motor_id] = count;
    encoder_count[motor_id] = 0;
    return count;
}

/** 返回 PID 最近一个 50 ms 周期读取到的脉冲数，不清零当前计数。 */
int32_t Encoder_Get_Last_Count(uint8_t motor_id)
{
    return motor_id < 2U ? encoder_last_count[motor_id] : 0;
}

/** 兼容原调试接口；GPIO 脉冲方案不提供 ADC 数值。 */
uint16_t Encoder_Get_ADC(uint8_t motor_id)
{
    (void)motor_id;
    return 0U;
}
