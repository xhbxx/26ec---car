#include "ti_msp_dl_config.h"

#include "Emm_V5.h"
#include "bianma.h"
#include "delay.h"
#include "oled.h"
#include "pid.h"
#include "uart.h"

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_PERIOD_MS              (20UL)
#define CONTROL_DT_S                   (0.020f)
#define SENSOR_TIMEOUT_MS              (300UL)
#define OLED_UPDATE_PERIOD_MS          (250UL)

/* 调速度环时改为0；速度环稳定后改回1，启用完整双闭环。 */
#define POSITION_LOOP_ENABLED          (1U)
#define SPEED_LOOP_TEST_TARGET         (20.0f)

#define POSITION_MIN                   (0U)
#define POSITION_MAX                   (500U)
#define POSITION_DEFAULT_TARGET        (240U)
#define POSITION_TARGET_STEP           (1)
#define POSITION_DEADBAND              (3.0f)

#define POSITION_KP                    (0.35f)
#define POSITION_KI                    (0.002f)
#define POSITION_KD                    (0.10f)
#define POSITION_MAX_SPEED             (80.0f)
#define POSITION_INTEGRAL_LIMIT        (400.0f)

#define SPEED_KP                       (0.080f)
#define SPEED_KI                       (0.012f)
#define SPEED_KD                       (0.001f)
/* 增大水管允许倾角，提高小球驱动力；电机移动速度仍保持30 RPM。 */
#define PIPE_MAX_ANGLE_DEG             (10.0f)
#define SPEED_INTEGRAL_LIMIT           (120.0f)
#define SPEED_FILTER_ALPHA             (0.20f)
#define PID_DERIVATIVE_ALPHA           (0.15f)

#define EMM_ADDRESS                    (1U)
#define EMM_MOVE_SPEED_RPM             (30U)
#define EMM_MOVE_ACCELERATION          (10U)
#define EMM_MIN_COMMAND_ANGLE_DEG      (0.12f)
#define EMM_DIRECTION_POSITIVE         (0U)
#define EMM_DIRECTION_NEGATIVE         (1U)

#define FRAME_SIZE                     (7U)
#define FRAME_SOF_1                    (0xAAU)
#define FRAME_SOF_2                    (0x55U)
#define FRAME_VERSION                  (0x01U)

volatile uint16_t current_position = 0U;
volatile uint16_t target_position = POSITION_DEFAULT_TARGET;
volatile float current_speed = 0.0f;
volatile float target_speed = 0.0f;
volatile float pipe_angle = 0.0f;
volatile uint32_t valid_frame_count = 0U;

static PID_TypeDef g_position_pid;
static PID_TypeDef g_speed_pid;
static volatile uint32_t g_milliseconds = 0U;
static uint32_t g_last_frame_ms = 0U;
static uint32_t g_last_control_ms = 0U;
static uint32_t g_last_oled_ms = 0U;
static uint32_t g_last_encoder_ms = 0U;
static uint16_t g_last_position = 0U;
static float g_last_commanded_angle = 0.0f;
static uint8_t g_frame[FRAME_SIZE];
static uint8_t g_frame_index = 0U;
static uint8_t g_position_valid = 0U;
static uint8_t g_motor_stopped = 1U;

/* [llm-pid-tuner] SETPOINT 文本指令接收状态。 */
static char g_llm_command[20];
static uint8_t g_llm_command_index = 0U;

/** [llm-pid-tuner] 完成 SysConfig 已创建的 UART1 轮询接收配置。 */
static void LLM_UART_Init(void)
{
    DL_UART_Main_setRXFIFOThreshold(
        LLM_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    NVIC_DisableIRQ(LLM_UART_INST_INT_IRQN);
}

/** 提供固定 1 ms 系统时间基准。 */
void SysTick_Handler(void)
{
    g_milliseconds++;
}

/** 计算传感器 7 字节帧使用的 CRC-8/ATM。 */
static uint8_t Sensor_CalculateCrc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    uint8_t index;

    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x80U) != 0U)
                ? (uint8_t)((crc << 1U) ^ 0x07U)
                : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

/**
 * 接收 AA 55 01 SEQ POS_L POS_H CRC；得到0~500的位置时更新反馈。
 */
static uint8_t Sensor_PushByte(uint8_t value)
{
    uint16_t position;

    if (g_frame_index == 0U) {
        if (value == FRAME_SOF_1) {
            g_frame[g_frame_index++] = value;
        }
        return 0U;
    }
    if (g_frame_index == 1U) {
        if (value == FRAME_SOF_2) {
            g_frame[g_frame_index++] = value;
        } else {
            g_frame_index = (value == FRAME_SOF_1) ? 1U : 0U;
        }
        return 0U;
    }

    g_frame[g_frame_index++] = value;
    if (g_frame_index < FRAME_SIZE) {
        return 0U;
    }
    g_frame_index = 0U;

    position = (uint16_t)g_frame[4] | ((uint16_t)g_frame[5] << 8U);
    if ((g_frame[2] != FRAME_VERSION) ||
        (Sensor_CalculateCrc8(g_frame, FRAME_SIZE - 1U) !=
         g_frame[FRAME_SIZE - 1U]) ||
        (position > POSITION_MAX)) {
        return 0U;
    }

    if (g_position_valid == 0U) {
        g_last_position = position;
    }
    current_position = position;
    g_position_valid = 1U;
    g_last_frame_ms = g_milliseconds;
    valid_frame_count++;
    return 1U;
}

/** 使用旋钮在0~500内调整目标位置，按下旋钮恢复目标240。 */
static void Target_PositionUpdate(void)
{
    int8_t rotation = Bianma_GetRotation();
    int32_t updated = (int32_t)target_position +
        (int32_t)rotation * POSITION_TARGET_STEP;

    if (updated < (int32_t)POSITION_MIN) {
        updated = POSITION_MIN;
    } else if (updated > (int32_t)POSITION_MAX) {
        updated = POSITION_MAX;
    }
    target_position = (uint16_t)updated;

    if (Bianma_Button_Pressed() != 0U) {
        target_position = POSITION_DEFAULT_TARGET;
    }
}

/**
 * 将内环输出的绝对管角转换为 Emm_V5 相对角度增量；限制机械角度并抑制碎小命令。
 */
static void Motor_SetAngle(float requested_angle)
{
    float delta;
    uint8_t direction;

    if (requested_angle > PIPE_MAX_ANGLE_DEG) {
        requested_angle = PIPE_MAX_ANGLE_DEG;
    } else if (requested_angle < -PIPE_MAX_ANGLE_DEG) {
        requested_angle = -PIPE_MAX_ANGLE_DEG;
    }

    delta = requested_angle - g_last_commanded_angle;
    if ((delta > -EMM_MIN_COMMAND_ANGLE_DEG) &&
        (delta < EMM_MIN_COMMAND_ANGLE_DEG)) {
        return;
    }

    direction = (delta >= 0.0f)
        ? EMM_DIRECTION_POSITIVE : EMM_DIRECTION_NEGATIVE;
    if (delta < 0.0f) {
        delta = -delta;
    }
    Emm_V5_Pos_Control_Angle(EMM_ADDRESS, direction,
        EMM_MOVE_SPEED_RPM, EMM_MOVE_ACCELERATION,
        delta, false, false);
    g_last_commanded_angle = requested_angle;
    g_motor_stopped = 0U;
}

/** [llm-pid-tuner] 发送无符号十进制整数，不依赖 printf 浮点库。 */
static void LLM_SendUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        UART_send_char(LLM_UART_INST, (uint8_t)'0');
        return;
    }
    while ((value != 0U) && (count < (uint8_t)sizeof(digits))) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count > 0U) {
        UART_send_char(LLM_UART_INST, (uint8_t)digits[--count]);
    }
}

/** [llm-pid-tuner] 发送带符号的两位小数，兼容 MSPM0 精简 C 库。 */
static void LLM_SendFixed2(float value)
{
    int32_t scaled;

    if (value < 0.0f) {
        UART_send_char(LLM_UART_INST, (uint8_t)'-');
        value = -value;
    }
    scaled = (int32_t)(value * 100.0f + 0.5f);
    LLM_SendUnsigned((uint32_t)(scaled / 100));
    UART_send_char(LLM_UART_INST, (uint8_t)'.');
    UART_send_char(LLM_UART_INST, (uint8_t)('0' + ((scaled / 10) % 10)));
    UART_send_char(LLM_UART_INST, (uint8_t)('0' + (scaled % 10)));
}

/** [llm-pid-tuner] 发送一行外环 PID 遥测 CSV。 */
static void LLM_UART_Report(void)
{
    const int32_t error = (int32_t)target_position -
        (int32_t)current_position;

    LLM_SendUnsigned(g_milliseconds);
    UART_send_char(LLM_UART_INST, ',');
    LLM_SendUnsigned(target_position);
    UART_send_char(LLM_UART_INST, ',');
    LLM_SendUnsigned(current_position);
    UART_send_char(LLM_UART_INST, ',');
    LLM_SendFixed2(pipe_angle);
    UART_send_char(LLM_UART_INST, ',');
    if (error < 0) {
        UART_send_char(LLM_UART_INST, '-');
        LLM_SendUnsigned((uint32_t)(-error));
    } else {
        LLM_SendUnsigned((uint32_t)error);
    }
    UART_send_char(LLM_UART_INST, ',');
    LLM_SendFixed2(g_position_pid.Kp);
    UART_send_char(LLM_UART_INST, ',');
    LLM_SendFixed2(g_position_pid.Ki);
    UART_send_char(LLM_UART_INST, ',');
    LLM_SendFixed2(g_position_pid.Kd);
    UART_send_char(LLM_UART_INST, '\n');
}

/** [llm-pid-tuner] 解析 SETPOINT:<value>，错误或越界指令静默丢弃。 */
static void LLM_UART_ParseSetpointByte(uint8_t received)
{
    uint8_t index;
    uint16_t value = 0U;
    uint8_t valid = 1U;

    if (received == '\r') {
        return;
    }
    if (received != '\n') {
        if (g_llm_command_index < (uint8_t)(sizeof(g_llm_command) - 1U)) {
            g_llm_command[g_llm_command_index++] = (char)received;
        } else {
            g_llm_command_index = 0U;
        }
        return;
    }

    g_llm_command[g_llm_command_index] = '\0';
    g_llm_command_index = 0U;
    if ((g_llm_command[0] != 'S') || (g_llm_command[1] != 'E') ||
        (g_llm_command[2] != 'T') || (g_llm_command[3] != 'P') ||
        (g_llm_command[4] != 'O') || (g_llm_command[5] != 'I') ||
        (g_llm_command[6] != 'N') || (g_llm_command[7] != 'T') ||
        (g_llm_command[8] != ':')) {
        return;
    }
    if (g_llm_command[9] == '\0') {
        return;
    }
    for (index = 9U; g_llm_command[index] != '\0'; index++) {
        if ((g_llm_command[index] < '0') ||
            (g_llm_command[index] > '9')) {
            valid = 0U;
            break;
        }
        value = (uint16_t)(value * 10U +
            (uint16_t)(g_llm_command[index] - '0'));
        if (value > POSITION_MAX) {
            valid = 0U;
            break;
        }
    }
    if (valid != 0U) {
        target_position = value;
    }
}

/** [llm-pid-tuner] 轮询调参串口，保证解析过程不阻塞控制任务。 */
static void LLM_UART_Poll(void)
{
    uint8_t received;

    while (DL_UART_Main_receiveDataCheck(LLM_UART_INST, &received)) {
        LLM_UART_ParseSetpointByte(received);
    }
}

/** 在OLED上显示带符号的三位整数，超出范围时限制到正负999。 */
static void OLED_ShowSigned3(uint8_t x, uint8_t y, float value)
{
    int32_t rounded = (value >= 0.0f)
        ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
    uint32_t magnitude;

    if (rounded > 999) {
        rounded = 999;
    } else if (rounded < -999) {
        rounded = -999;
    }
    OLED_ShowString(x, y, (u8 *)((rounded < 0) ? "-" : "+"), 12U);
    magnitude = (rounded < 0)
        ? (uint32_t)(-rounded) : (uint32_t)rounded;
    OLED_ShowNum((uint8_t)(x + 6U), y, magnitude, 3U, 12U);
}

/** 每250ms显示位置、速度、管角和外环PID，不占用20ms控制周期频率。 */
static void OLED_ShowControlInfo(void)
{
    uint32_t kp = (uint32_t)(g_position_pid.Kp * 1000.0f + 0.5f);
    uint32_t ki = (uint32_t)(g_position_pid.Ki * 1000.0f + 0.5f);
    uint32_t kd = (uint32_t)(g_position_pid.Kd * 1000.0f + 0.5f);

    if (kp > 999U) kp = 999U;
    if (ki > 999U) ki = 999U;
    if (kd > 999U) kd = 999U;

    OLED_ShowString(0U, 0U, (u8 *)"T:", 12U);
    OLED_ShowNum(12U, 0U, target_position, 3U, 12U);
    OLED_ShowString(30U, 0U, (u8 *)" C:", 12U);
    OLED_ShowNum(48U, 0U, current_position, 3U, 12U);
    OLED_ShowString(72U, 0U,
        (u8 *)((g_position_valid != 0U) ? "V" : "X"), 12U);

    OLED_ShowString(0U, 16U, (u8 *)"TS:", 12U);
    OLED_ShowSigned3(18U, 16U, target_speed);
    OLED_ShowString(42U, 16U, (u8 *)" CS:", 12U);
    OLED_ShowSigned3(66U, 16U, current_speed);

    OLED_ShowString(0U, 32U, (u8 *)"A:", 12U);
    OLED_ShowSigned3(12U, 32U, pipe_angle * 100.0f);
    OLED_ShowString(36U, 32U, (u8 *)"/100deg", 12U);

    OLED_ShowString(0U, 48U, (u8 *)"P:", 12U);
    OLED_ShowNum(12U, 48U, kp, 3U, 12U);
    OLED_ShowString(30U, 48U, (u8 *)" I:", 12U);
    OLED_ShowNum(48U, 48U, ki, 3U, 12U);
    OLED_ShowString(66U, 48U, (u8 *)" D:", 12U);
    OLED_ShowNum(84U, 48U, kd, 3U, 12U);
    OLED_Refresh();
}

/** 复位双环状态和速度估计，并立即停止尚未完成的电机运动。 */
static void Ball_ControlStop(void)
{
    PID_Reset(&g_position_pid);
    PID_Reset(&g_speed_pid);
    current_speed = 0.0f;
    target_speed = 0.0f;
    pipe_angle = 0.0f;
    g_last_position = current_position;
    g_position_valid = 0U;
    if (g_motor_stopped == 0U) {
        Emm_V5_Stop_Now(EMM_ADDRESS, false);
        g_motor_stopped = 1U;
    }
}

/**
 * 固定20 ms执行位置外环和速度内环，并将目标管角交给已有电机驱动。
 */
static void Ball_ControlUpdate(void)
{
    float raw_speed = ((float)current_position - (float)g_last_position) /
        CONTROL_DT_S;
    float position_error = (float)target_position -
        (float)current_position;

    g_last_position = current_position;
    current_speed += SPEED_FILTER_ALPHA * (raw_speed - current_speed);

#if POSITION_LOOP_ENABLED
    target_speed = PID_Update(&g_position_pid,
        (float)target_position, (float)current_position);
    if ((position_error >= -POSITION_DEADBAND) &&
        (position_error <= POSITION_DEADBAND)) {
        target_speed = 0.0f;
        g_position_pid.integral = 0.0f;
    }
#else
    /* 单独调内环时使用固定目标速度，不执行位置PID。 */
    target_speed = SPEED_LOOP_TEST_TARGET;
    (void)position_error;
#endif

    pipe_angle = PID_Update(&g_speed_pid, target_speed, current_speed);
    Motor_SetAngle(pipe_angle);
}

/** 初始化双环、串口、旋钮和 Emm_V5，并运行固定周期控制任务。 */
int main(void)
{
    SYSCFG_DL_init();
    LLM_UART_Init();
    Bianma_Init();
    NVIC_DisableIRQ(PRINT_INST_INT_IRQN);

    PID_Init(&g_position_pid, POSITION_KP, POSITION_KI, POSITION_KD,
        CONTROL_DT_S, POSITION_MAX_SPEED, POSITION_INTEGRAL_LIMIT,
        PID_DERIVATIVE_ALPHA);
    PID_Init(&g_speed_pid, SPEED_KP, SPEED_KI, SPEED_KD,
        CONTROL_DT_S, PIPE_MAX_ANGLE_DEG, SPEED_INTEGRAL_LIMIT,
        PID_DERIVATIVE_ALPHA);

    OLED_Init();
    OLED_ShowControlInfo();
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    delay_ms(500U);
    Emm_V5_En_Control(EMM_ADDRESS, true, false);
    delay_ms(10U);

    while (1) {
        uint8_t received;
        uint32_t now;

        LLM_UART_Poll();
        while (UART_read_received_byte(&received) != 0U) {
            (void)Sensor_PushByte(received);
        }
        now = g_milliseconds;
        /* 每1 ms读取一次旋钮，使20次按钮消抖计数对应真实20 ms。 */
        if ((uint32_t)(now - g_last_encoder_ms) >= 1U) {
            g_last_encoder_ms = now;
            Target_PositionUpdate();
        }

        if ((uint32_t)(now - g_last_control_ms) >= CONTROL_PERIOD_MS) {
            g_last_control_ms += CONTROL_PERIOD_MS;

            if ((g_position_valid != 0U) &&
                ((uint32_t)(now - g_last_frame_ms) <= SENSOR_TIMEOUT_MS)) {
                Ball_ControlUpdate();
            } else {
                Ball_ControlStop();
            }

            /* [llm-pid-tuner] 每个20ms控制周期上报一次外环数据。 */
            LLM_UART_Report();
        }

        if ((uint32_t)(now - g_last_oled_ms) >= OLED_UPDATE_PERIOD_MS) {
            g_last_oled_ms = now;
            OLED_ShowControlInfo();
        }
    }
}
