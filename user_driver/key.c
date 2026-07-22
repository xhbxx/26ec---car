#include "key.h"

#include "encoder.h"

/* 兼容工程中仍保留的按键模块；双路PWM主程序不使用该状态变量。 */
volatile int status = 0;

/** 读取指定按键引脚的电平，高电平返回1。 */
uint8_t get_key_state(uint32_t key)
{
    return (DL_GPIO_readPins(KEY_PORT, key) & key) != 0U ? 1U : 0U;
}

/** 共享处理按键和两路编码器中断，左右轮脉冲分别计数。 */
void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
    case ENCODER_LEFT_PULSE_IIDX:
        Encoder_Record_Pulse(0U);
        break;
    default:
        break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
    case ENCODER_RIGHT_PULSE_IIDX:
        Encoder_Record_Pulse(1U);
        break;
    case KEY_KEY9_IIDX:
        status = (status + 1) % 3;
        break;
    case KEY_KEY10_IIDX:
        status = (status + 2) % 3;
        break;
    default:
        break;
    }
}
