#include "ti_msp_dl_config.h"

#include "car_config.h"
#include "chassis_controller.h"
#include "encoder.h"
#include "imu_heading.h"
#include "line_sensor.h"
#include "motor.h"
#include "oled.h"
#include "atk_ms6dsv.h"

#include <stdint.h>

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    uint8_t raw_pressed;
    uint8_t stable_pressed;
} ButtonState;

static volatile uint32_t g_milliseconds;
static ButtonState g_mode_button;
static ButtonState g_start_button;
static uint8_t g_mode_index;
static uint8_t g_mode_confirmed;
static const uint8_t g_modes[3] = {2U, 4U, 5U};

void SysTick_Handler(void)
{
    g_milliseconds++;
}

void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB,
        ENCODER_LEFT_PULSE_PIN | ENCODER_RIGHT_PULSE_PIN);

    if ((status & ENCODER_LEFT_PULSE_PIN) != 0U) {
        Encoder_Record_Pulse(0U);
    }
    if ((status & ENCODER_RIGHT_PULSE_PIN) != 0U) {
        Encoder_Record_Pulse(1U);
    }
    DL_GPIO_clearInterruptStatus(GPIOB, status);
}

static void button_init(ButtonState *button, GPIO_Regs *port,
    uint32_t pin)
{
    uint8_t pressed =
        (DL_GPIO_readPins(port, pin) & pin) == 0U ? 1U : 0U;
    button->port = port;
    button->pin = pin;
    button->raw_pressed = pressed;
    button->stable_pressed = pressed;
}

static uint8_t button_pressed_event(ButtonState *button)
{
    uint8_t pressed =
        (DL_GPIO_readPins(button->port, button->pin) & button->pin) == 0U
        ? 1U : 0U;

    if (pressed != button->raw_pressed) {
        button->raw_pressed = pressed;
        if (button->stable_pressed != pressed) {
            button->stable_pressed = pressed;
            return pressed;
        }
    }
    return 0U;
}

static uint8_t selected_mode(void)
{
    return g_modes[g_mode_index];
}

static void display_clear_row(uint8_t y)
{
    OLED_ShowString(0U, y, (u8 *)"                     ", 12U);
}

static uint32_t display_speed_mmps(int32_t speed_x10)
{
    if (speed_x10 < 0) {
        speed_x10 = -speed_x10;
    }
    return (uint32_t)(speed_x10 / 10L);
}

static void display_update(void)
{
    int32_t angle_mdeg = ImuHeading_GetAngleMdeg();
    uint32_t angle_deg;
    uint32_t elapsed_tenths;

    /* 陀螺仪完成静止零偏校准前，只显示等待提示。 */
    if (ImuHeading_IsReady() == 0U) {
        display_clear_row(0U);
        display_clear_row(16U);
        OLED_ShowString(0U, 16U, (u8 *)"LOADING", 12U);
        display_clear_row(32U);
        display_clear_row(48U);
        OLED_Refresh();
        return;
    }

    if (angle_mdeg < 0) {
        angle_deg = (uint32_t)(-angle_mdeg / 1000L);
    } else {
        angle_deg = (uint32_t)(angle_mdeg / 1000L);
    }

    if (g_mode_confirmed == 0U) {
        display_clear_row(0U);
        OLED_ShowString(0U, 0U, (u8 *)"SELECT MODE", 12U);
        display_clear_row(16U);
        OLED_ShowString(0U, 16U, (u8 *)"MODE ", 12U);
        OLED_ShowNum(36U, 16U, selected_mode(), 1U, 12U);
        display_clear_row(32U);
        OLED_ShowString(0U, 32U, (u8 *)"PA23: NEXT", 12U);
        display_clear_row(48U);
        OLED_ShowString(0U, 48U, (u8 *)"PB18: START", 12U);
        OLED_Refresh();
        return;
    }

    display_clear_row(0U);
    OLED_ShowString(0U, 0U, (u8 *)"L SET:", 12U);
    OLED_ShowNum(36U, 0U, display_speed_mmps(
        motor_get_target_mmps_x10(MOTOR_ID_A)), 4U, 12U);
    OLED_ShowString(60U, 0U, (u8 *)" ACT:", 12U);
    OLED_ShowNum(90U, 0U, display_speed_mmps(
        motor_get_actual_mmps_x10(MOTOR_ID_A)), 4U, 12U);

    display_clear_row(16U);
    OLED_ShowString(0U, 16U, (u8 *)"R SET:", 12U);
    OLED_ShowNum(36U, 16U, display_speed_mmps(
        motor_get_target_mmps_x10(MOTOR_ID_B)), 4U, 12U);
    OLED_ShowString(60U, 16U, (u8 *)" ACT:", 12U);
    OLED_ShowNum(90U, 16U, display_speed_mmps(
        motor_get_actual_mmps_x10(MOTOR_ID_B)), 4U, 12U);

    display_clear_row(32U);
    OLED_ShowString(0U, 32U, (u8 *)"M:", 12U);
    OLED_ShowNum(12U, 32U, ChassisController_GetTaskMode(), 1U, 12U);
    OLED_ShowString(18U, 32U, (u8 *)" ST:", 12U);
    OLED_ShowNum(42U, 32U, ChassisController_GetState(), 1U, 12U);
    if (ChassisController_GetTaskMode() == 2U) {
        elapsed_tenths = ChassisController_GetElapsedMs(
            g_milliseconds) / 100U;
        OLED_ShowString(54U, 32U, (u8 *)" T:", 12U);
        OLED_ShowNum(72U, 32U, elapsed_tenths / 10U, 2U, 12U);
        OLED_ShowString(84U, 32U, (u8 *)".", 12U);
        OLED_ShowNum(90U, 32U, elapsed_tenths % 10U, 1U, 12U);
        OLED_ShowString(96U, 32U, (u8 *)"S", 12U);
    } else {
        OLED_ShowString(54U, 32U, (u8 *)" IMU:", 12U);
        OLED_ShowNum(84U, 32U, ATK_MS6DSV_GetStatus(), 1U, 12U);
    }

    display_clear_row(48U);
    OLED_ShowString(0U, 48U, (u8 *)"GYRO:", 12U);
    OLED_ShowString(36U, 48U, (u8 *)(angle_mdeg < 0 ? "-" : "+"), 12U);
    OLED_ShowNum(42U, 48U, angle_deg, 3U, 12U);
    OLED_ShowString(66U, 48U, (u8 *)"DEG", 12U);
    OLED_Refresh();
}

static void process_buttons(uint32_t now_ms)
{
    ChassisState state = ChassisController_GetState();
    uint8_t idle = (state == CHASSIS_WAIT) ||
        (state == CHASSIS_COMPLETE) || (state == CHASSIS_ERROR);

    /* 校准完成前不允许切换模式或启动车辆。 */
    if (ImuHeading_IsReady() == 0U) {
        return;
    }

    if ((button_pressed_event(&g_mode_button) != 0U) &&
        (idle != 0U)) {
        g_mode_index = (uint8_t)((g_mode_index + 1U) % 3U);
        g_mode_confirmed = 0U;
        display_update();
    }

    if ((button_pressed_event(&g_start_button) != 0U) &&
        (idle != 0U)) {
        if (ChassisController_Start(selected_mode(), now_ms) != 0U) {
            g_mode_confirmed = 1U;
            display_update();
        }
    }
}

int main(void)
{
    LineSensorData line = {{0U, 0U, 0U, 0U}, 0U, 0U, 0U, 0U, 0};
    uint32_t last_control_ms = 0U;
    uint32_t last_display_ms = 0U;

    SYSCFG_DL_init();
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

    OLED_Init();
    button_init(&g_mode_button, BUTTONS_MODE_PORT, BUTTONS_MODE_PIN);
    button_init(&g_start_button, BUTTONS_START_PORT, BUTTONS_START_PIN);
    display_update();

    motor_init(MOTOR_ID_A);
    motor_init(MOTOR_ID_B);
    Encoder_Init();
    LineSensor_Init();
    ChassisController_Init();

    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
    NVIC_ClearPendingIRQ(MOTOR_PID_INST_INT_IRQN);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
    DL_Timer_startCounter(MOTOR_PID_INST);

    ImuHeading_Init(g_milliseconds);
    display_update();

    while (1) {
        uint32_t now_ms = g_milliseconds;

        ImuHeading_Update(now_ms);
        process_buttons(now_ms);

        if ((uint32_t)(now_ms - last_control_ms) >=
            CONTROL_UPDATE_PERIOD_MS) {
            last_control_ms += CONTROL_UPDATE_PERIOD_MS;
            LineSensor_Read(&line);
            ChassisController_Update(now_ms, &line);
        }

        if ((uint32_t)(now_ms - last_display_ms) >=
            DISPLAY_UPDATE_PERIOD_MS) {
            last_display_ms = now_ms;
            display_update();
        }
    }
}
