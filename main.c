#include "ti_msp_dl_config.h"

#include "grayscale_sensor.h"
#include "line_tracking.h"
#include "motor.h"

/**
 * 灰度循迹主程序。
 * 所有外设只在进入主循环前初始化一次；主循环持续读取八路灰度值并更新两路电机。
 */
int main(void)
{
    uint16_t sensor_values[GRAYSCALE_SENSOR_CHANNELS] = {0U};

    SYSCFG_DL_init();
    motor_init(MOTOR_ID_A);
    motor_init(MOTOR_ID_B);
    Grayscale_Sensor_Init();

    while (1) {
        Grayscale_Sensor_Read_All(sensor_values);
        Line_Tracking_Update(sensor_values);

        /* 约5 ms更新一次，兼顾循迹响应速度与传感器信号稳定性。 */
        DL_Common_delayCycles(CPUCLK_FREQ / 200U);
    }
}
