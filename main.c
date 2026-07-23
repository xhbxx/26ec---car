#include "ti_msp_dl_config.h"

#include "grayscale_sensor.h"
#include "line_tracking.h"
#include "motor.h"
#include "encoder.h"
#include "oled.h"
#include "key.h"

/* Display a signed speed value on one OLED row. */
static void OLED_ShowSignedPercent(uint8_t x, uint8_t y, int32_t value)
{
    uint32_t magnitude;

    OLED_ShowString(x, y, (u8 *)(value < 0 ? "-" : "+"), 16U);
    magnitude = (uint32_t)(value < 0 ? -value : value);
    if (magnitude > 999U) {
        magnitude = 999U;
    }
    OLED_ShowNum((uint8_t)(x + 16U), y, (u32)magnitude, 3U, 16U);
}

/* Show target and measured wheel speed in the same unit: percent. */
static void OLED_ShowMotorPercent(uint8_t y, uint8_t motor_id)
{
    int32_t actual_speed = motor_get_actual_speed_percent(motor_id);

    OLED_ShowString(0U, y, (u8 *)"T:", 16U);
    OLED_ShowSignedPercent(16U, y,
        motor_get_target_speed_percent(motor_id));
    OLED_ShowString(64U, y, (u8 *)"S:", 16U);
    if (actual_speed < 0) {
        actual_speed = -actual_speed;
    }
    if (actual_speed > 999) {
        actual_speed = 999;
    }
    OLED_ShowNum(80U, y, (u32)actual_speed, 3U, 16U);
}

/* Show P/I/D; all three values use 100x fixed-point scaling. */
static void OLED_ShowPidGains(void)
{
    OLED_ShowString(0U, 32U, (u8 *)"P:", 8U);
    OLED_ShowNum(8U, 32U, (u32)motor_get_pid_kp(), 3U, 8U);
    OLED_ShowString(28U, 32U, (u8 *)"I:", 8U);
    OLED_ShowNum(36U, 32U, (u32)motor_get_pid_ki(), 3U, 8U);
    OLED_ShowString(56U, 32U, (u8 *)"D:", 8U);
    OLED_ShowNum(64U, 32U, (u32)motor_get_pid_kd(), 3U, 8U);
}

/**
 * 灰度循迹主程序。
 * 所有外设只在进入主循环前初始化一次；主循环持续读取八路灰度值并更新两路电机。
 */
int main(void)
{
    uint16_t sensor_values[GRAYSCALE_SENSOR_CHANNELS] = {0U};
    u8 sensor_text[GRAYSCALE_SENSOR_CHANNELS + 1U] = "00000000";
    uint8_t oled_update_count = 0U;
    uint8_t sensor_channel;
    
    SYSCFG_DL_init();

    /* 系统初始化后 PB22 才被配置为 GPIO 输出，此时再置高点亮指示灯。 */
    
    /* 先点亮 OLED，再启动电机和中断，便于独立判断屏幕是否正常。 */
    OLED_Init();
    DL_GPIO_setPins(LED_PORT, LED_B22_PIN);
    OLED_ShowMotorPercent(0U, MOTOR_ID_A);
    OLED_ShowMotorPercent(16U, MOTOR_ID_B);
    /* 下两行按 0~7 顺序显示检测状态：1=黑线，0=未检测到。 */
    OLED_ShowPidGains();
    OLED_ShowString(0U, 48U, sensor_text, 16U);
    OLED_Refresh();
    
    motor_init(MOTOR_ID_A);
    motor_init(MOTOR_ID_B);
    /* SysConfig leaves the 50 ms PID timer stopped; start it once here. */
    DL_Timer_startCounter(MOTOR_PID_INST);
    Encoder_Init();
    /* Both encoder inputs are on GPIOB: PB8 left and PB9 right. */
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
    NVIC_ClearPendingIRQ(MOTOR_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
    Grayscale_Sensor_Init();

    while (1) {
        Key_Process();
        Grayscale_Sensor_Read_All(sensor_values);
        Line_Tracking_Update(sensor_values);

        /* oled.c 刷新整屏较慢，每约 500 ms 更新一次，避免拖慢循迹。 */
        oled_update_count++;
        if (oled_update_count >= 100U) {
            OLED_ShowMotorPercent(0U, MOTOR_ID_A);
            OLED_ShowMotorPercent(16U, MOTOR_ID_B);
            OLED_ShowPidGains();
            for (sensor_channel = 0U;
                 sensor_channel < GRAYSCALE_SENSOR_CHANNELS;
                 sensor_channel++) {
                sensor_text[sensor_channel] =
                    (u8)(sensor_values[sensor_channel] != 0U ? '1' : '0');
            }
            sensor_text[GRAYSCALE_SENSOR_CHANNELS] = '\0';
            OLED_ShowString(0U, 48U, sensor_text, 16U);
            OLED_Refresh();
            oled_update_count = 0U;
        }

        /* 约5 ms更新一次，兼顾循迹响应速度与传感器信号稳定性。 */
        DL_Common_delayCycles(CPUCLK_FREQ / 200U);
    }
}
