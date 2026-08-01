#include "ti_msp_dl_config.h"

#include "car_config.h"
#include "chassis_controller.h"
#include "encoder.h"
#include "imu_heading.h"
#include "line_sensor.h"
#include "motor.h"
#include "oled.h"
#include "atk_ms6dsv.h"
#include "telemetry_protocol.h"

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
static uint8_t g_telemetry_frame[TELEMETRY_FRAME_SIZE];
static volatile uint8_t g_telemetry_tx_index;
static volatile uint8_t g_telemetry_tx_busy;
static uint16_t g_telemetry_sequence;
static int32_t g_telemetry_encoder_accel_mmps2;
static int32_t g_telemetry_last_actual_mmps;
static int32_t g_telemetry_last_target_mmps;
static uint32_t g_telemetry_last_encoder_ms;
static uint32_t g_telemetry_last_target_ms;
static uint8_t g_display_refresh_page;
static uint8_t g_display_refresh_pending;

static void telemetry_service(void);

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

static int16_t telemetry_limit_i16(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

static void telemetry_put_u16(uint8_t offset, uint16_t value)
{
    g_telemetry_frame[offset] = (uint8_t)(value & 0xFFU);
    g_telemetry_frame[offset + 1U] = (uint8_t)(value >> 8U);
}

static void telemetry_put_u32(uint8_t offset, uint32_t value)
{
    g_telemetry_frame[offset] = (uint8_t)(value & 0xFFU);
    g_telemetry_frame[offset + 1U] = (uint8_t)((value >> 8U) & 0xFFU);
    g_telemetry_frame[offset + 2U] = (uint8_t)((value >> 16U) & 0xFFU);
    g_telemetry_frame[offset + 3U] = (uint8_t)(value >> 24U);
}

static uint16_t telemetry_crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t index;

    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 1U) != 0U
                ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static uint8_t telemetry_state_flags(ChassisState state, uint32_t now_ms)
{
    uint8_t flags = 0U;

    if (ImuHeading_IsReady() != 0U) {
        flags |= TELEMETRY_FLAG_IMU_READY;
    }
    if (ImuHeading_IsOnline() != 0U) {
        flags |= TELEMETRY_FLAG_IMU_ONLINE;
    }
    if (ChassisController_IsRunning() != 0U) {
        flags |= TELEMETRY_FLAG_RUNNING;
        if (ChassisController_GetElapsedMs(now_ms) >=
            (uint32_t)MOTOR_SPEED_SAMPLE_MS) {
            flags |= TELEMETRY_FLAG_ENCODER_VALID;
        }
    }
    if ((state == CHASSIS_STRAIGHT_AB) ||
        (state == CHASSIS_STRAIGHT_CD) ||
        (state == CHASSIS_FINAL_APPROACH)) {
        flags |= TELEMETRY_FLAG_STRAIGHT;
    }
    if ((state == CHASSIS_CURVE_BC) || (state == CHASSIS_CURVE_DA)) {
        flags |= TELEMETRY_FLAG_CURVE;
    }
    if (state == CHASSIS_STOPPING) {
        flags |= TELEMETRY_FLAG_STOPPING;
    }
    return flags;
}

static void telemetry_start_frame(uint32_t now_ms)
{
    ChassisState state;
    int32_t left_target_mmps;
    int32_t right_target_mmps;
    int32_t left_actual_mmps;
    int32_t right_actual_mmps;
    int32_t target_mmps;
    int32_t actual_mmps;
    int32_t target_accel_mmps2 = 0;
    int32_t steering_mmps;
    uint32_t elapsed_ms;
    uint16_t crc;

    if (g_telemetry_tx_busy != 0U) {
        return;
    }

    left_target_mmps = (motor_get_target_mmps_x10(MOTOR_ID_A) *
        CHASSIS_LEFT_FORWARD_SIGN) / 10L;
    right_target_mmps = (motor_get_target_mmps_x10(MOTOR_ID_B) *
        CHASSIS_RIGHT_FORWARD_SIGN) / 10L;
    left_actual_mmps = (motor_get_actual_mmps_x10(MOTOR_ID_A) *
        CHASSIS_LEFT_FORWARD_SIGN) / 10L;
    right_actual_mmps = (motor_get_actual_mmps_x10(MOTOR_ID_B) *
        CHASSIS_RIGHT_FORWARD_SIGN) / 10L;
    target_mmps = (left_target_mmps + right_target_mmps) / 2L;
    actual_mmps = (left_actual_mmps + right_actual_mmps) / 2L;
    steering_mmps = (left_target_mmps - right_target_mmps) / 2L;

    elapsed_ms = now_ms - g_telemetry_last_target_ms;
    if ((g_telemetry_last_target_ms != 0U) && (elapsed_ms != 0U)) {
        target_accel_mmps2 = (int32_t)(
            ((int64_t)(target_mmps - g_telemetry_last_target_mmps) *
             1000LL) / elapsed_ms);
    }
    g_telemetry_last_target_ms = now_ms;
    g_telemetry_last_target_mmps = target_mmps;

    elapsed_ms = now_ms - g_telemetry_last_encoder_ms;
    if ((g_telemetry_last_encoder_ms == 0U) ||
        (elapsed_ms >= (uint32_t)MOTOR_SPEED_SAMPLE_MS)) {
        if ((g_telemetry_last_encoder_ms != 0U) && (elapsed_ms != 0U)) {
            g_telemetry_encoder_accel_mmps2 = (int32_t)(
                ((int64_t)(actual_mmps - g_telemetry_last_actual_mmps) *
                 1000LL) / elapsed_ms);
        }
        g_telemetry_last_encoder_ms = now_ms;
        g_telemetry_last_actual_mmps = actual_mmps;
    }

    state = ChassisController_GetState();
    g_telemetry_frame[0] = 0xAAU;
    g_telemetry_frame[1] = 0x55U;
    g_telemetry_frame[2] = TELEMETRY_PROTOCOL_VERSION;
    g_telemetry_frame[3] = TELEMETRY_FRAME_SIZE;
    telemetry_put_u16(4U, g_telemetry_sequence++);
    telemetry_put_u32(6U, now_ms);
    g_telemetry_frame[10] = ChassisController_GetTaskMode();
    g_telemetry_frame[11] = (uint8_t)state;
    g_telemetry_frame[12] = telemetry_state_flags(state, now_ms);
    g_telemetry_frame[13] = 0U;
    telemetry_put_u16(14U, (uint16_t)telemetry_limit_i16(target_mmps));
    telemetry_put_u16(16U, (uint16_t)telemetry_limit_i16(actual_mmps));
    telemetry_put_u16(18U, (uint16_t)telemetry_limit_i16(left_actual_mmps));
    telemetry_put_u16(20U, (uint16_t)telemetry_limit_i16(right_actual_mmps));
    telemetry_put_u16(22U,
        (uint16_t)telemetry_limit_i16(target_accel_mmps2));
    telemetry_put_u16(24U,
        (uint16_t)telemetry_limit_i16(g_telemetry_encoder_accel_mmps2));
    telemetry_put_u16(26U, (uint16_t)ImuHeading_GetAccelXMg());
    telemetry_put_u16(28U, (uint16_t)ImuHeading_GetAccelYMg());
    telemetry_put_u16(30U, (uint16_t)ImuHeading_GetAccelZMg());
    telemetry_put_u32(32U, (uint32_t)ImuHeading_GetRateMdps());
    telemetry_put_u32(36U, (uint32_t)ImuHeading_GetAngleMdeg());
    telemetry_put_u32(40U,
        (uint32_t)ChassisController_GetDistanceMm());
    telemetry_put_u16(44U, (uint16_t)telemetry_limit_i16(steering_mmps));
    crc = telemetry_crc16(&g_telemetry_frame[2], 44U);
    telemetry_put_u16(46U, crc);

    g_telemetry_tx_index = 0U;
    g_telemetry_tx_busy = 1U;
    /* 先装满 FIFO，再由 TX 中断连续发送剩余字节。 */
    telemetry_service();
    if (g_telemetry_tx_busy != 0U) {
        DL_UART_Main_enableInterrupt(TELEMETRY_UART_INST,
            DL_UART_MAIN_INTERRUPT_TX);
    }
}

static void telemetry_service(void)
{
    while ((g_telemetry_tx_busy != 0U) &&
        (DL_UART_Main_isTXFIFOFull(TELEMETRY_UART_INST) == false)) {
        DL_UART_Main_transmitData(TELEMETRY_UART_INST,
            g_telemetry_frame[g_telemetry_tx_index++]);
        if (g_telemetry_tx_index >= TELEMETRY_FRAME_SIZE) {
            g_telemetry_tx_busy = 0U;
            DL_UART_Main_disableInterrupt(TELEMETRY_UART_INST,
                DL_UART_MAIN_INTERRUPT_TX);
            break;
        }
    }
}

void TELEMETRY_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(TELEMETRY_UART_INST)) {
    case DL_UART_MAIN_IIDX_TX:
        telemetry_service();
        break;
    default:
        break;
    }
}

static void display_clear_row(uint8_t y)
{
    OLED_ShowString(0U, y, (u8 *)"                     ", 12U);
}

/*
 * 只标记显存需要发送到OLED。整屏I2C刷新约需100ms，会阻塞10ms遥测任务；
 * 主循环随后在UART帧发送完成的空隙中逐页刷新，避免接收端50ms超时。
 */
static void display_request_refresh(void)
{
    g_display_refresh_page = 0U;
    g_display_refresh_pending = 1U;
}

static void display_refresh_service(void)
{
    if ((g_display_refresh_pending == 0U) ||
        (g_telemetry_tx_busy != 0U)) {
        return;
    }

    OLED_RefreshPage(g_display_refresh_page++);
    if (g_display_refresh_page >= 8U) {
        g_display_refresh_pending = 0U;
    }
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
        display_request_refresh();
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
        display_request_refresh();
        return;
    }

    /* Mode 2 只显示比赛计时；任务结束后 GetElapsedMs 返回冻结时间。 */
    if (ChassisController_GetTaskMode() == 2U) {
        elapsed_tenths = ChassisController_GetElapsedMs(
            g_milliseconds) / 100U;
        display_clear_row(0U);
        display_clear_row(16U);
        OLED_ShowString(0U, 16U, (u8 *)"TIME:", 12U);
        OLED_ShowNum(36U, 16U, elapsed_tenths / 10U, 2U, 12U);
        OLED_ShowString(48U, 16U, (u8 *)".", 12U);
        OLED_ShowNum(54U, 16U, elapsed_tenths % 10U, 1U, 12U);
        OLED_ShowString(60U, 16U, (u8 *)"S", 12U);
        display_clear_row(32U);
        display_clear_row(48U);
        display_request_refresh();
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
    OLED_ShowString(54U, 32U, (u8 *)" IMU:", 12U);
    OLED_ShowNum(84U, 32U, ATK_MS6DSV_GetStatus(), 1U, 12U);

    display_clear_row(48U);
    OLED_ShowString(0U, 48U, (u8 *)"GYRO:", 12U);
    OLED_ShowString(36U, 48U, (u8 *)(angle_mdeg < 0 ? "-" : "+"), 12U);
    OLED_ShowNum(42U, 48U, angle_deg, 3U, 12U);
    OLED_ShowString(66U, 48U, (u8 *)"DEG", 12U);
    display_request_refresh();
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
    uint32_t last_telemetry_ms = 0U;

    SYSCFG_DL_init();
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

    DL_UART_Main_disableInterrupt(TELEMETRY_UART_INST,
        DL_UART_MAIN_INTERRUPT_TX);
    NVIC_ClearPendingIRQ(TELEMETRY_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(TELEMETRY_UART_INST_INT_IRQN);

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

        /* 遥测优先：先按10ms周期启动帧，再利用发送空隙刷新一页OLED。 */
        if ((uint32_t)(now_ms - last_telemetry_ms) >=
            TELEMETRY_PERIOD_MS) {
            last_telemetry_ms += TELEMETRY_PERIOD_MS;
            telemetry_start_frame(now_ms);
        }
        display_refresh_service();
    }
}
