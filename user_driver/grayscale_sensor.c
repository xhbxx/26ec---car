#include "grayscale_sensor.h"

/** 按当前 80 MHz 系统时钟产生微秒级延时。 */
static void grayscale_delay_us(uint32_t microseconds)
{
    DL_Common_delayCycles((CPUCLK_FREQ / 1000000U) * microseconds);
}

/** AD0 为最低位、AD1 为次位、AD2 为最高位，选择 0~7 通道。 */
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

/** 直接读取地猛星 PA18/OUT；模块检测到黑线时返回 1。 */
static uint16_t grayscale_read_out(void)
{
    return (DL_GPIO_readPins(GRAYSCALE_PORT, GRAYSCALE_OUT_PIN) != 0U)
        ? 1U : 0U;
}

/** GPIO 方向和初始电平已经由 SysConfig 初始化。 */
void Grayscale_Sensor_Init(void)
{
}

/** 完全沿用 Grayscale_Read 示例：逐路选择，等待 50 us 后读取一次。 */
void Grayscale_Sensor_Read_All(uint16_t *sensor_values)
{
    uint8_t channel;

    if (sensor_values == 0) {
        return;
    }

    for (channel = 0U; channel < GRAYSCALE_SENSOR_CHANNELS; channel++) {
        grayscale_select_channel(channel);
        grayscale_delay_us(50U);
        sensor_values[channel] = grayscale_read_out();
    }
}

/** 读取指定通道；通道号必须为 0~7。 */
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNELS) {
        return 0U;
    }

    grayscale_select_channel(channel);
    grayscale_delay_us(50U);
    return grayscale_read_out();
}
