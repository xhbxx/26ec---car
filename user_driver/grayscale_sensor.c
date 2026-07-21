#include "grayscale_sensor.h"

/** 等待灰度模块通道切换稳定，当前系统时钟下约为指定微秒数。 */
static void grayscale_delay_us(uint32_t microseconds)
{
    while (microseconds-- > 0U) {
        DL_Common_delayCycles(CPUCLK_FREQ / 1000000U);
    }
}

/** 通过AD0、AD1、AD2选择八路灰度模块的一个输出通道。 */
static void grayscale_select_channel(uint8_t channel)
{
    if ((channel & 0x01U) != 0U) {
        DL_GPIO_setPins(GRAYSCALE_PORT, GRAYSCALE_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GRAYSCALE_PORT, GRAYSCALE_AD0_PIN);
    }

    if ((channel & 0x02U) != 0U) {
        DL_GPIO_setPins(GRAYSCALE_PORT, GRAYSCALE_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GRAYSCALE_PORT, GRAYSCALE_AD1_PIN);
    }

    if ((channel & 0x04U) != 0U) {
        DL_GPIO_setPins(GRAYSCALE_PORT, GRAYSCALE_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GRAYSCALE_PORT, GRAYSCALE_AD2_PIN);
    }
}

/** 初始化灰度通道选择状态；GPIO方向已由SysConfig统一配置。 */
void Grayscale_Sensor_Init(void)
{
    DL_GPIO_clearPins(GRAYSCALE_PORT,
        GRAYSCALE_AD0_PIN | GRAYSCALE_AD1_PIN | GRAYSCALE_AD2_PIN);
    grayscale_delay_us(50U);
}

/** 依次读取0～7号传感器，数组下标顺序必须与车头左到右的安装顺序一致。 */
void Grayscale_Sensor_Read_All(uint16_t *sensor_values)
{
    uint8_t channel;

    if (sensor_values == 0) {
        return;
    }

    for (channel = 0U; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        grayscale_select_channel(channel);
        grayscale_delay_us(50U);
        sensor_values[channel] =
            (DL_GPIO_readPins(GRAYSCALE_PORT, GRAYSCALE_OUT_PIN) != 0U) ? 1U : 0U;
    }
}

/** 读取指定的一路灰度数字量；通道参数超出0～7时返回0。 */
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNELS) {
        return 0U;
    }

    grayscale_select_channel(channel);
    grayscale_delay_us(50U);
    return (DL_GPIO_readPins(GRAYSCALE_PORT, GRAYSCALE_OUT_PIN) != 0U) ? 1U : 0U;
}
