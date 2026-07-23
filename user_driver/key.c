#include "key.h"

#include "encoder.h"
#include "mpu6050.h"

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
    const uint32_t interrupt_pins = MPU_INT_INT_PIN |
        ENCODER_LEFT_PULSE_PIN | ENCODER_RIGHT_PULSE_PIN |
        KEY_KEY9_PIN | KEY_KEY10_PIN;
    const uint32_t pending =
        DL_GPIO_getEnabledInterruptStatus(GPIOB, interrupt_pins);

    /* Test every pending bit so simultaneous GPIOB events are not dropped. */
    if ((pending & MPU_INT_INT_PIN) != 0U) {
        MPU6050_OnInterrupt();
    }
    if ((pending & ENCODER_LEFT_PULSE_PIN) != 0U) {
        Encoder_Record_Pulse(0U);
    }
    if ((pending & ENCODER_RIGHT_PULSE_PIN) != 0U) {
        Encoder_Record_Pulse(1U);
    }
    if ((pending & KEY_KEY9_PIN) != 0U) {
        status = (status + 1) % 3;
    }
    if ((pending & KEY_KEY10_PIN) != 0U) {
        status = (status + 2) % 3;
    }

    DL_GPIO_clearInterruptStatus(GPIOB, pending);
}
