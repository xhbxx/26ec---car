#include "ti_msp_dl_config.h"

#include "encoder.h"
#include "grayscale_sensor.h"
#include "line_tracking.h"
#include "motor.h"
#include "mpu6050.h"
#include "oled.h"

static uint8_t g_led_blink_ticks = 0U;
static uint8_t g_led_last_present = 1U;

/** 用符号和三位数字显示目标/实际速度百分比。 */
static void OLED_ShowSignedPercent(uint8_t x, uint8_t y, int32_t value)
{
    uint32_t magnitude;

    magnitude = (uint32_t)(value < 0 ? -value : value);
    if (magnitude > 999U) {
        magnitude = 999U;
    }
    OLED_ShowString(x, y, (u8 *)(value < 0 ? "-" : "+"), 12U);
    OLED_ShowNum((uint8_t)(x + 6U), y, magnitude, 3U, 12U);
}

/** 用符号和五位数字显示陀螺仪原始量。 */
static void OLED_ShowSignedRaw(uint8_t x, uint8_t y, int32_t value)
{
    uint32_t magnitude = (uint32_t)(value < 0 ? -value : value);

    if (magnitude > 99999U) {
        magnitude = 99999U;
    }
    OLED_ShowString(x, y, (u8 *)(value < 0 ? "-" : "+"), 12U);
    OLED_ShowNum((uint8_t)(x + 6U), y, magnitude, 5U, 12U);
}

/** 第一行显示左轮，第二行显示右轮；T=目标，A=实际。 */
static void OLED_ShowMotorSpeeds(void)
{
    OLED_ShowString(0U, 0U, (u8 *)"LT", 12U);
    OLED_ShowSignedPercent(12U, 0U,
        motor_get_target_speed_percent(MOTOR_ID_A));
    OLED_ShowString(60U, 0U, (u8 *)"LA", 12U);
    OLED_ShowSignedPercent(72U, 0U,
        motor_get_actual_speed_percent(MOTOR_ID_A));

    OLED_ShowString(0U, 12U, (u8 *)"RT", 12U);
    OLED_ShowSignedPercent(12U, 12U,
        motor_get_target_speed_percent(MOTOR_ID_B));
    OLED_ShowString(60U, 12U, (u8 *)"RA", 12U);
    OLED_ShowSignedPercent(72U, 12U,
        motor_get_actual_speed_percent(MOTOR_ID_B));
}

/** 显示GY-6500直行控制数据：G原始值、B零偏、E误差、C差速。 */
static void OLED_ShowGyroStraightData(void)
{
    OLED_ShowString(0U, 24U, (u8 *)"G", 12U);
    OLED_ShowSignedRaw(6U, 24U, Line_Tracking_GetGyroRaw());
    OLED_ShowString(60U, 24U, (u8 *)"B", 12U);
    OLED_ShowSignedRaw(66U, 24U, Line_Tracking_GetGyroBias());

    OLED_ShowString(0U, 36U, (u8 *)"E", 12U);
    OLED_ShowSignedRaw(6U, 36U, Line_Tracking_GetGyroError());
    OLED_ShowString(60U, 36U, (u8 *)"C", 12U);
    OLED_ShowSignedPercent(66U, 36U, Line_Tracking_GetGyroCorrection());
    OLED_ShowString(108U, 36U, (u8 *)"R", 12U);
    OLED_ShowNum(114U, 36U, Line_Tracking_IsGyroReady(), 1U, 12U);
}

/** 灰度从有线变为丢线或重新找线时，短暂点亮指示灯。 */
static void track_led_task(uint8_t line_present)
{
    if (line_present != g_led_last_present) {
        g_led_blink_ticks = 1U;
        DL_GPIO_setPins(LED_PORT, LED_B22_PIN);
        g_led_last_present = line_present;
    }
    if (g_led_blink_ticks > 0U) {
        g_led_blink_ticks--;
        if (g_led_blink_ticks == 0U) {
            DL_GPIO_clearPins(LED_PORT, LED_B22_PIN);
        }
    }
}

/** 初始化一次外设，持续更新MPU6050、灰度循迹和OLED。 */
int main(void)
{
    uint16_t sensor_values[GRAYSCALE_SENSOR_CHANNELS] = {0U};
    u8 sensor_text[GRAYSCALE_SENSOR_CHANNELS + 1U] = "00000000";
    uint8_t oled_update_count = 0U;
    uint8_t sensor_channel;

    SYSCFG_DL_init();
    OLED_Init();
    motor_init(MOTOR_ID_A);
    motor_init(MOTOR_ID_B);
    DL_Timer_startCounter(MOTOR_PID_INST);
    Encoder_Init();

    /* PB1、PB8、PB9共用GPIOB中断入口。 */
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_ClearPendingIRQ(MOTOR_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    /* MPU6050: PB3=SDA, PB2=SCL, PB1=INT。 */
    (void)MPU6050_Init();
    Grayscale_Sensor_Init();

    OLED_ShowMotorSpeeds();
    OLED_ShowGyroStraightData();
    
    OLED_ShowString(0U, 48U, sensor_text, 16U);
    OLED_Refresh();

    while (1) {
        uint8_t line_present;

        (void)MPU6050_Update();
        Grayscale_Sensor_Read_All(sensor_values);
        line_present = Line_Tracking_Update(sensor_values);
        track_led_task(line_present);

        oled_update_count++;
        if (oled_update_count >= 100U) {
           
            OLED_ShowMotorSpeeds();
            OLED_ShowGyroStraightData();
           
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

        DL_Common_delayCycles(CPUCLK_FREQ / 200U);
    }
}
