#include "ti_msp_dl_config.h"

#include "grayscale_sensor.h"
#include "line_tracking.h"
#include "motor.h"
#include "encoder.h"
#include "oled.h"

/**
 * 灰度循迹主程序。
 * 所有外设只在进入主循环前初始化一次；主循环持续读取八路灰度值并更新两路电机。
 */
int main(void)
{
    uint16_t sensor_values[GRAYSCALE_SENSOR_CHANNELS] = {0U};
    uint8_t oled_update_count = 0U;
    
    SYSCFG_DL_init();

    /* 系统初始化后 PB22 才被配置为 GPIO 输出，此时再置高点亮指示灯。 */
    
    /* 先点亮 OLED，再启动电机和中断，便于独立判断屏幕是否正常。 */
    OLED_Init();
    DL_GPIO_setPins(LED_PORT, LED_B22_PIN);
    OLED_ShowString(0U, 0U, (u8 *)"L:", 16U);
    OLED_ShowNum(24U, 0U, 0U, 5U, 16U);
    OLED_ShowString(0U, 24U, (u8 *)"R:", 16U);
    OLED_ShowNum(24U, 24U, 0U, 5U, 16U);
    OLED_Refresh();

    motor_init(MOTOR_ID_A);
    motor_init(MOTOR_ID_B);
    Encoder_Init();
    NVIC_ClearPendingIRQ(ENCODER_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_ClearPendingIRQ(MOTOR_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
    Grayscale_Sensor_Init();

    while (1) {
        Grayscale_Sensor_Read_All(sensor_values);
        Line_Tracking_Update(sensor_values);

        /* oled.c 刷新整屏较慢，每约 500 ms 更新一次，避免拖慢循迹。 */
        oled_update_count++;
        if (oled_update_count >= 100U) {
            OLED_ShowNum(24U, 0U,
                (u32)Encoder_Get_Last_Count(0U), 5U, 16U);
            OLED_ShowNum(24U, 24U,
                (u32)Encoder_Get_Last_Count(1U), 5U, 16U);
            OLED_Refresh();
            oled_update_count = 0U;
        }

        /* 约5 ms更新一次，兼顾循迹响应速度与传感器信号稳定性。 */
        DL_Common_delayCycles(CPUCLK_FREQ / 200U);
    }
}
