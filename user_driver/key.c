#include "key.h"

/* 兼容工程中仍保留的按键模块；双路PWM主程序不使用该状态变量。 */
volatile int status = 0;

/** 读取指定按键引脚的电平，高电平返回1。 */
uint8_t get_key_state(uint32_t key)
{
    return (DL_GPIO_readPins(KEY_PORT, key) & key) != 0U ? 1U : 0U;
}

/** 仅处理保留的两个按键中断，不再引用已移除的编码器引脚。 */
void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
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
