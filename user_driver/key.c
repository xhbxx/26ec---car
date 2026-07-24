#include "key.h"

#include "encoder.h"
#include "mpu6050.h"
#include "motor.h"

/* 兼容工程中仍保留的按键模块；双路PWM主程序不使用该状态变量。 */
volatile int status = 0;

/** Poll and debounce four active-low PID tuning keys; each press changes one step. */
void Key_Process(void)
{
#if defined(KEY_KP_INC_PIN) && defined(KEY_KP_DEC_PIN) && \
    defined(KEY_KI_INC_PIN) && defined(KEY_KI_DEC_PIN)
    static uint8_t filter[4] = {0U, 0U, 0U, 0U};
    static uint8_t latched[4] = {0U, 0U, 0U, 0U};
    const uint32_t pins[4] = {
        KEY_KP_INC_PIN, KEY_KP_DEC_PIN, KEY_KI_INC_PIN, KEY_KI_DEC_PIN
    };
    uint8_t index;

    for (index = 0U; index < 4U; index++) {
        if ((DL_GPIO_readPins(KEY_PORT, pins[index]) & pins[index]) == 0U) {
            if (filter[index] < 3U) {
                filter[index]++;
            }
            if ((filter[index] >= 3U) && (latched[index] == 0U)) {
                if (index == 0U) {
                    motor_adjust_pid_kp(1);
                } else if (index == 1U) {
                    motor_adjust_pid_kp(-1);
                } else if (index == 2U) {
                    motor_adjust_pid_ki(1);
                } else {
                    motor_adjust_pid_ki(-1);
                }
                latched[index] = 1U;
            }
        } else {
            filter[index] = 0U;
            latched[index] = 0U;
        }
    }
#endif
}

/** 读取指定按键引脚的电平，高电平返回1。 */
uint8_t get_key_state(uint32_t key)
{
    return (DL_GPIO_readPins(KEY_PORT, key) & key) != 0U ? 1U : 0U;
}

/** 共享处理按键和两路编码器中断，左右轮脉冲分别计数。 */
void GROUP1_IRQHandler(void)
{
    const uint32_t interrupt_pins = ENCODER_LEFT_PULSE_PIN |
        ENCODER_RIGHT_PULSE_PIN | MPU_INT_INT_PIN;
    const uint32_t pending =
        DL_GPIO_getEnabledInterruptStatus(GPIOB, interrupt_pins);

    /* Test every pending bit so simultaneous GPIOB events are not dropped. */
    if ((pending & ENCODER_LEFT_PULSE_PIN) != 0U) {
        Encoder_Record_Pulse(0U);
    }
    if ((pending & ENCODER_RIGHT_PULSE_PIN) != 0U) {
        Encoder_Record_Pulse(1U);
    }
    if ((pending & MPU_INT_INT_PIN) != 0U) {
        MPU6050_OnInterrupt();
    }
    DL_GPIO_clearInterruptStatus(GPIOB, pending);
}
