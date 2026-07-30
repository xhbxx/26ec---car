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
/*
 * 速度比位置更容易过期：超过该时间没有新视觉速度样本，就不能继续用于
 * 位置预测和速度环，否则旧速度会让控制器误以为小球仍在运动。
 */
#define SPEED_SAMPLE_STALE_MS          (120UL)
#define OLED_UPDATE_PERIOD_MS          (250UL)
#define MOTOR_FEEDBACK_QUERY_MS        (40UL)
#define MOTOR_COMMAND_PERIOD_MS        (40UL)
#define MOTOR_FEEDBACK_TIMEOUT_MS      (200UL)
/* PID只允许小角度运动，反馈相对启动零点超出该范围时视为异常帧。 */
#define MOTOR_FEEDBACK_ANGLE_LIMIT_DEG (15.0f)

/*
 * 双环调试开关：
 * 1U = 位置外环和速度内环同时工作，用于正常运行。
 * 0U = 暂时关闭位置外环，只给速度内环固定目标，便于先单独调好速度环。
 */
#define POSITION_LOOP_ENABLED          (1U)

/* 仅在 POSITION_LOOP_ENABLED=0U 时使用，表示速度环测试目标，单位为位置值/秒。 */
#define SPEED_LOOP_TEST_TARGET         (20.0f)

/* 视觉位置数据及旋钮目标值允许的最小值。 */
#define POSITION_MIN                   (0U)
/* 视觉位置数据及旋钮目标值允许的最大值。 */
#define POSITION_MAX                   (500U)
/* 上电后的默认目标位置；按下旋钮也会恢复到这个值。 */
#define POSITION_DEFAULT_TARGET        (250U)
/* 旋钮每转动一格，目标位置增加或减少的数值。 */
#define POSITION_TARGET_STEP           (1)
/* 位置死区：|目标位置-当前位置|不超过该值时，目标速度置零，防止目标附近来回动作。 */
/* 只在非常接近目标时锁定；误差超过2立即恢复缓慢调整。 */
#define POSITION_DEADBAND              (1.0f)
#define POSITION_DEADBAND_EXIT         (2.0f)
/* 只有球速足够低并进入±1时才判定到达，防止高速穿过目标时过早停止控制。 */
#define POSITION_LOCK_SPEED            (2.0f)
/* 视觉计算速度在静止时仍有约±10噪声，该范围在控制计算中按0处理。 */
#define SPEED_ZERO_DEADBAND             (10.0f)
/* 根据当前速度预测约0.1秒后的球位置，用于在到达目标前提前制动。 */
#define POSITION_LOOKAHEAD_S            (0.1f)
/* 进入该距离后限制目标速度，使靠近目标阶段保持缓慢调整。 */
#define POSITION_SLOW_ZONE             (80.0f)
#define POSITION_NEAR_MIN_SPEED        (1.5f)
#define POSITION_NEAR_SPEED_SLOPE      (0.15f)
/* 每20ms允许目标速度改变的最大值，避免进入死区时从PID输出硬切到0。 */
#define TARGET_SPEED_SLEW_PER_CYCLE    (2.0f)
/*
 * 制动时允许目标速度更快地减小或反向。
 * 这不是提高正常追踪速度，而是缩短“球已经有惯性、控制器却还在慢慢撤销旧速度”的时间。
 */
#define TARGET_SPEED_BRAKE_SLEW        (4.0f)
/* 进入位置死区后每20ms回零的最大速度变化量，专门消除setpoint硬切阶跃。 */
#define TARGET_SPEED_STOP_SLEW         (0.75f)

/*
 * 位置外环 PID：输入目标位置和当前位置，输出 target_speed。
 * 调参时先令 POSITION_KI=0，调整 KP、KD，最后再少量增加 KI。
 */
/* 位置比例系数：误差越大，要求的目标速度越大；过大会过冲和来回摆动，过小则响应慢。 */
#define POSITION_KP                    (0.30f)
/* 位置积分系数：消除长期位置偏差；过大会积分累积并导致明显过冲，通常只使用很小数值。 */
#define POSITION_KI                    (0.00f)
/* 位置微分系数：根据误差变化提前减速、增加阻尼；过大会放大位置噪声并造成电机抖动。 */
#define POSITION_KD                    (0.00f)
/* 位置外环最大输出，即 target_speed 的绝对值上限；越大允许小球移动得越快，也越容易冲过目标。 */
#define POSITION_MAX_SPEED             (40.0f)
/* 位置环积分累计上限，用于防止长时间大误差造成积分饱和；不是速度或角度上限。 */
#define POSITION_INTEGRAL_LIMIT        (400.0f)

/*
 * 速度内环 PID：输入 target_speed 和滤波后的 current_speed，输出 pipe_angle。
 * 它决定水管需要倾斜多少来使小球速度跟随位置外环的要求。
 */
/* 速度比例系数：速度误差对应的即时倾角；增大可提高动作幅度，过大会造成速度震荡。 */
#define SPEED_KP                       (0.040f)
/* 速度积分系数：补偿摩擦、坡度等造成的长期速度不足；过大会持续加大倾角并导致过冲。 */
#define SPEED_KI                       (0.00f)
/* 速度微分系数：抑制速度突然变化；速度反馈噪声较大，因此通常只能使用很小数值。 */
#define SPEED_KD                       (0.00f)
/* 速度内环最终输出的机械管角上限，单位为度；同时限制正、负两个方向为 ±该数值。 */
#define PIPE_MAX_ANGLE_DEG             (4.0f)
/*
 * 负管角方向的机构力度补偿：1.0表示不补偿，数值越大，靠近500一侧的回拉幅度越大。
 * 补偿后的角度仍会被 PIPE_MAX_ANGLE_DEG 限制，不会突破机械角度上限。
 */
#define PIPE_NEGATIVE_ANGLE_GAIN       (0.75f)
/* 往0方向使用负管角，单独限制该方向的最大幅度，避免下降方向动作过大。 */
#define PIPE_NEGATIVE_MAX_ANGLE_DEG    (3.0f)
/* 靠近目标时分方向限制管角；正方向需要更强制动力，负方向保持原限制。 */
#define PIPE_NEAR_ZONE                 (60.0f)
#define PIPE_NEAR_POSITIVE_MAX_DEG     (1.8f)
#define PIPE_NEAR_NEGATIVE_MAX_DEG     (1.2f)
/*
 * 实测静摩擦不对称：往500方向约1°才能可靠启动，往0方向约0.7°。
 * 两个数值只在球静止且仍在目标外时使用，运动后立即交回速度PID。
 */
#define PIPE_RESTART_POSITIVE_DEG      (0.85f)
#define PIPE_RESTART_NEGATIVE_DEG      (1.20f)
/* 每20ms允许管角目标改变的最大角度，限制电机和水管的瞬时动作。 */
#define PIPE_ANGLE_SLEW_PER_CYCLE      (0.15f)
/* 死区内速度低于该值时认为小球基本静止，管角平滑回到机械零角。 */
#define BALL_STOP_SPEED_THRESHOLD      (2.0f)
/* 死区内每周期保留的积分比例，逐渐卸掉旧方向积分而不是瞬间清零。 */
#define PID_INTEGRAL_DECAY             (0.90f)
/* 速度环积分累计上限，防止小球长期不动时积分不断增加并突然产生过大倾角。 */
#define SPEED_INTEGRAL_LIMIT           (120.0f)
/* 速度一阶低通系数：越大越跟手但噪声越明显，越小越平滑但速度反馈延迟越大，范围0~1。 */
#define SPEED_FILTER_ALPHA             (0.20f)
/* 两个 PID 共用的微分低通系数：越大越灵敏，越小越平滑，范围0~1。 */
#define PID_DERIVATIVE_ALPHA           (0.15f)

#define EMM_ADDRESS                    (1U)
#define EMM_MOVE_SPEED_RPM             (30U)
#define EMM_MOVE_ACCELERATION          (10U)
#define EMM_MIN_COMMAND_ANGLE_DEG      (0.12f)
/* 电机正负管角对应的方向；若机构安装方向变化，应成对交换0U和1U。 */
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
static uint32_t g_last_position_sample_ms = 0U;
static uint8_t g_speed_sample_initialized = 0U;
static float g_motor_requested_angle = 0.0f;
static float g_motor_actual_angle = 0.0f;
static float g_motor_zero_absolute_angle = 0.0f;
static uint32_t g_last_motor_feedback_ms = 0U;
static uint32_t g_last_motor_query_ms = 0U;
static uint32_t g_last_motor_command_ms = 0U;
static uint8_t g_motor_feedback_valid = 0U;
static uint8_t g_motor_zero_captured = 0U;
static uint8_t g_motor_rx_frame[8];
static uint8_t g_motor_rx_index = 0U;
static uint8_t g_oled_refresh_page = 8U;
static uint8_t g_frame[FRAME_SIZE];
static uint8_t g_frame_index = 0U;
static uint8_t g_position_valid = 0U;
static uint8_t g_motor_stopped = 1U;
static uint8_t g_position_locked = 0U;
static uint16_t g_position_lock_target = 0U;

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
    uint32_t now;

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

    now = g_milliseconds;
    if (g_speed_sample_initialized == 0U) {
        g_last_position = position;
        g_last_position_sample_ms = now;
        current_speed = 0.0f;
        g_speed_sample_initialized = 1U;
    } else {
        uint32_t elapsed_ms = (uint32_t)(now - g_last_position_sample_ms);

        if ((elapsed_ms > 0U) && (elapsed_ms <= SENSOR_TIMEOUT_MS)) {
            float raw_speed = ((float)position - (float)g_last_position) *
                1000.0f / (float)elapsed_ms;
            current_speed += SPEED_FILTER_ALPHA *
                (raw_speed - current_speed);
            g_last_position = position;
            g_last_position_sample_ms = now;
        } else if (elapsed_ms > SENSOR_TIMEOUT_MS) {
            /* 间隔过长时旧速度已无意义，从当前帧重新建立差分基准。 */
            current_speed = 0.0f;
            g_last_position = position;
            g_last_position_sample_ms = now;
        }
        /* elapsed_ms==0时保留旧基准，避免同一毫秒内处理积压帧而丢失总位移。 */
    }
    current_position = position;
    g_position_valid = 1U;
    g_last_frame_ms = now;
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

/** 记录PID要求的管角；真正的电机命令由Motor_ControlService结合实时位置发送。 */
static void Motor_SetAngle(float requested_angle)
{
    if (requested_angle > PIPE_MAX_ANGLE_DEG) {
        requested_angle = PIPE_MAX_ANGLE_DEG;
    } else if (requested_angle < -PIPE_MAX_ANGLE_DEG) {
        requested_angle = -PIPE_MAX_ANGLE_DEG;
    }
    g_motor_requested_angle = requested_angle;
}

/**
 * 解析电机实时位置返回帧：Addr 36 Sign Position[4] 6B。
 * Emm固件位置单位为一圈65536，首次有效反馈作为水管机械零角。
 */
static void Motor_PushFeedbackByte(uint8_t value)
{
    uint32_t raw_position;
    float absolute_angle;
    float relative_angle;

    if (g_motor_rx_index == 0U) {
        if (value == EMM_ADDRESS) {
            g_motor_rx_frame[g_motor_rx_index++] = value;
        }
        return;
    }
    if (g_motor_rx_index == 1U) {
        if (value == 0x36U) {
            g_motor_rx_frame[g_motor_rx_index++] = value;
        } else {
            g_motor_rx_index = (value == EMM_ADDRESS) ? 1U : 0U;
        }
        return;
    }

    g_motor_rx_frame[g_motor_rx_index++] = value;
    if (g_motor_rx_index < (uint8_t)sizeof(g_motor_rx_frame)) {
        return;
    }
    g_motor_rx_index = 0U;

    if ((g_motor_rx_frame[7] != 0x6BU) ||
        (g_motor_rx_frame[2] > 1U)) {
        return;
    }

    raw_position = ((uint32_t)g_motor_rx_frame[3] << 24U) |
        ((uint32_t)g_motor_rx_frame[4] << 16U) |
        ((uint32_t)g_motor_rx_frame[5] << 8U) |
        (uint32_t)g_motor_rx_frame[6];
    absolute_angle = (float)raw_position * 360.0f / 65536.0f;
    if (g_motor_rx_frame[2] != 0U) {
        absolute_angle = -absolute_angle;
    }

    if (g_motor_zero_captured == 0U) {
        /*
         * 电机上电时水管所在位置就是机械静止位置。
         * 第一次有效反馈直接作为基准角，后续A/M均显示相对该位置的角度。
         */
        g_motor_zero_absolute_angle = absolute_angle;
        g_motor_zero_captured = 1U;
    }

    /* 将驱动器坐标换算回PID使用的正负管角坐标。 */
    relative_angle = absolute_angle - g_motor_zero_absolute_angle;
    if (EMM_DIRECTION_POSITIVE != 0U) {
        relative_angle = -relative_angle;
    }

    /*
     * 当前机械管角命令只有几度，瞬间出现±9.99°以上通常是串口错帧或
     * 符号/圈数跳变。异常反馈不参与电机闭环，防止M跳到999后误发命令。
     */
    if ((relative_angle > MOTOR_FEEDBACK_ANGLE_LIMIT_DEG) ||
        (relative_angle < -MOTOR_FEEDBACK_ANGLE_LIMIT_DEG)) {
        return;
    }
    g_motor_actual_angle = relative_angle;
    g_last_motor_feedback_ms = g_milliseconds;
    g_motor_feedback_valid = 1U;
}

/**
 * 轮询PA31电机反馈、周期查询实时位置，并按实际角度发送绝对位置目标。
 * 绝对目标不会像连续相对增量那样因执行延迟而重复累加。
 */
static void Motor_ControlService(uint32_t now)
{
    uint8_t received;

    while (Motor_UART_ReadByte(&received) != 0U) {
        Motor_PushFeedbackByte(received);
    }

    if ((g_motor_feedback_valid != 0U) &&
        ((uint32_t)(now - g_last_motor_feedback_ms) >
         MOTOR_FEEDBACK_TIMEOUT_MS)) {
        g_motor_feedback_valid = 0U;
    }

    if (((uint32_t)(now - g_last_motor_query_ms) >=
         MOTOR_FEEDBACK_QUERY_MS) &&
        ((uint32_t)(now - g_last_motor_command_ms) >= 5U)) {
        g_last_motor_query_ms = now;
        Emm_V5_Read_Sys_Params(EMM_ADDRESS, S_CPOS);
    }

    if ((g_motor_feedback_valid == 0U) ||
        ((uint32_t)(now - g_last_motor_feedback_ms) >
         MOTOR_FEEDBACK_TIMEOUT_MS) ||
        ((uint32_t)(now - g_last_motor_command_ms) <
         MOTOR_COMMAND_PERIOD_MS)) {
        return;
    }

    if ((g_motor_requested_angle - g_motor_actual_angle >
         -EMM_MIN_COMMAND_ANGLE_DEG) &&
        (g_motor_requested_angle - g_motor_actual_angle <
         EMM_MIN_COMMAND_ANGLE_DEG)) {
        return;
    }

    {
        float driver_target = g_motor_zero_absolute_angle +
            ((EMM_DIRECTION_POSITIVE == 0U) ?
                g_motor_requested_angle : -g_motor_requested_angle);
        uint8_t direction = 0U;

        if (driver_target < 0.0f) {
            driver_target = -driver_target;
            direction = 1U;
        }
        Emm_V5_Pos_Control_Angle(EMM_ADDRESS, direction,
            EMM_MOVE_SPEED_RPM, EMM_MOVE_ACCELERATION,
            driver_target, true, false);
        g_last_motor_command_ms = now;
        g_motor_stopped = 0U;
    }
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
    OLED_ShowString(36U, 32U, (u8 *)" M:", 12U);
    if (g_motor_feedback_valid != 0U) {
        OLED_ShowSigned3(54U, 32U, g_motor_actual_angle * 100.0f);
    } else {
        OLED_ShowString(54U, 32U, (u8 *)"----", 12U);
    }

    OLED_ShowString(0U, 48U, (u8 *)"P:", 12U);
    OLED_ShowNum(12U, 48U, kp, 3U, 12U);
    OLED_ShowString(30U, 48U, (u8 *)" I:", 12U);
    OLED_ShowNum(48U, 48U, ki, 3U, 12U);
    OLED_ShowString(66U, 48U, (u8 *)" D:", 12U);
    OLED_ShowNum(84U, 48U, kd, 3U, 12U);
    /* 这里只更新显存；主循环每个控制周期刷新一页，避免整屏阻塞控制。 */
    g_oled_refresh_page = 0U;
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
    g_speed_sample_initialized = 0U;
    g_position_valid = 0U;
    g_position_locked = 0U;
    /* 反馈丢失时保持当前机械角，不再继续追踪新的PID角度。 */
    if (g_motor_feedback_valid != 0U) {
        g_motor_requested_angle = g_motor_actual_angle;
    }
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
    float position_error = (float)target_position -
        (float)current_position;
    float predicted_position;
    float desired_target_speed;
    float desired_pipe_angle;
    float change;
    float absolute_position_error;
    float absolute_current_speed;
    float control_speed;
    float near_speed_limit;
    float target_speed_slew;
    uint32_t speed_sample_age_ms;
    uint8_t in_deadband;

    /*
     * 新位置帧超过120 ms未到达时，旧速度已经不能代表小球当前运动状态。
     * 立即将速度反馈归零，使预测位置退回当前位置，并让控制器重新调整，
     * 避免在距离目标约“旧速度×预测时间”的位置永久停住。
     */
    speed_sample_age_ms = (uint32_t)(g_milliseconds -
        g_last_position_sample_ms);
    if ((g_speed_sample_initialized != 0U) &&
        (speed_sample_age_ms > SPEED_SAMPLE_STALE_MS)) {
        current_speed = 0.0f;
    }

    /*
     * OLED仍显示原始滤波速度current_speed；PID使用去除静止噪声后的control_speed。
     * 否则球明明静止，±10左右的视觉速度噪声会阻止近距离启动补偿生效。
     */
    control_speed = current_speed;
    if ((control_speed >= -SPEED_ZERO_DEADBAND) &&
        (control_speed <= SPEED_ZERO_DEADBAND)) {
        control_speed = 0.0f;
    }

    absolute_position_error = (position_error < 0.0f)
        ? -position_error : position_error;
    absolute_current_speed = (control_speed < 0.0f)
        ? -control_speed : control_speed;

    /*
     * 用当前球速预测一小段时间后的落点，使水管在球真正到达目标前开始减速。
     * 对预测位置限幅，避免视觉速度偶发尖峰直接产生极端的反向控制量。
     */
    predicted_position = (float)current_position +
        control_speed * POSITION_LOOKAHEAD_S;
    if (predicted_position > 600.0f) {
        predicted_position = 600.0f;
    } else if (predicted_position < -100.0f) {
        predicted_position = -100.0f;
    }

    /* 修改目标位置时立即退出旧目标锁定；同一目标使用2/4滞回避免边界抖动。 */
    if ((g_position_locked != 0U) &&
        (g_position_lock_target != target_position)) {
        g_position_locked = 0U;
    }
    if (g_position_locked != 0U) {
        if (absolute_position_error > POSITION_DEADBAND_EXIT) {
            g_position_locked = 0U;
        }
    } else if ((absolute_position_error <= POSITION_DEADBAND) &&
        (absolute_current_speed <= POSITION_LOCK_SPEED)) {
        /* 只有位置接近且球速已经足够低，才认为真正到达目标。 */
        g_position_locked = 1U;
        g_position_lock_target = target_position;
    }
    in_deadband = g_position_locked;

#if POSITION_LOOP_ENABLED
    desired_target_speed = PID_UpdateConditional(&g_position_pid,
        (float)target_position, predicted_position,
        (uint8_t)(in_deadband == 0U));
    if (in_deadband != 0U) {
        desired_target_speed = 0.0f;
        PID_DecayIntegral(&g_position_pid, PID_INTEGRAL_DECAY);
    } else if (absolute_position_error < POSITION_SLOW_ZONE) {
        /*
         * 靠近目标后按实际距离动态限制速度：仍然允许缓慢微调，但不会用远距离时的大速度冲向目标。
         */
        near_speed_limit = POSITION_NEAR_MIN_SPEED +
            absolute_position_error * POSITION_NEAR_SPEED_SLOPE;
        if (desired_target_speed > near_speed_limit) {
            desired_target_speed = near_speed_limit;
        } else if (desired_target_speed < -near_speed_limit) {
            desired_target_speed = -near_speed_limit;
        }
    }
#else
    /* 单独调内环时使用固定目标速度，不执行位置PID。 */
    desired_target_speed = SPEED_LOOP_TEST_TARGET;
    (void)position_error;
    in_deadband = 0U;
#endif

    /* 对位置环输出做斜率限制，避免进入死区时目标速度发生阶跃。 */
    target_speed_slew = (in_deadband != 0U)
        ? TARGET_SPEED_STOP_SLEW : TARGET_SPEED_SLEW_PER_CYCLE;

    /*
     * 当预测结果要求减速或反向时，使用更快的制动斜率；正常加速仍保持原来的柔和斜率。
     */
    if (((target_speed > 0.0f) &&
         (desired_target_speed < target_speed)) ||
        ((target_speed < 0.0f) &&
         (desired_target_speed > target_speed))) {
        target_speed_slew = TARGET_SPEED_BRAKE_SLEW;
    }
    change = desired_target_speed - target_speed;
    if (change > target_speed_slew) {
        change = target_speed_slew;
    } else if (change < -target_speed_slew) {
        change = -target_speed_slew;
    }
    target_speed += change;

    if ((in_deadband != 0U) &&
        (absolute_current_speed <= BALL_STOP_SPEED_THRESHOLD)) {
        /* 小球基本静止后让水管回到零角，同时逐步卸掉旧方向速度积分。 */
        PID_DecayIntegral(&g_speed_pid, PID_INTEGRAL_DECAY);
        desired_pipe_angle = 0.0f;
    } else {
        desired_pipe_angle = PID_UpdateConditional(&g_speed_pid,
            target_speed, control_speed,
            (uint8_t)(in_deadband == 0U));
    }

    /* 往0方向使用负管角：降低该方向增益并单独限幅，避免动作幅度过大。 */
    if (desired_pipe_angle < 0.0f) {
        desired_pipe_angle *= PIPE_NEGATIVE_ANGLE_GAIN;
        if (desired_pipe_angle < -PIPE_NEGATIVE_MAX_ANGLE_DEG) {
            desired_pipe_angle = -PIPE_NEGATIVE_MAX_ANGLE_DEG;
        }
    }

    /*
     * 所有方向增益处理完成后，再检查最终管角是否足以克服静摩擦。
     * 这样负方向不会在达到启动角后又被0.75倍缩小到无法运动。
     */
    if ((in_deadband == 0U) &&
        (absolute_current_speed <= BALL_STOP_SPEED_THRESHOLD)) {
        if ((target_speed > 0.0f) &&
            (desired_pipe_angle < PIPE_RESTART_POSITIVE_DEG)) {
            desired_pipe_angle = PIPE_RESTART_POSITIVE_DEG;
        } else if ((target_speed < 0.0f) &&
            (desired_pipe_angle > -PIPE_RESTART_NEGATIVE_DEG)) {
            desired_pipe_angle = -PIPE_RESTART_NEGATIVE_DEG;
        }
    }

    /*
     * 目标附近只允许小角度加速和制动。速度误差即使因惯性变得很大，
     * 也不能直接输出大角度反打，从根源上限制来回大幅摆动。
     */
    if (absolute_position_error < PIPE_NEAR_ZONE) {
        if (desired_pipe_angle > PIPE_NEAR_POSITIVE_MAX_DEG) {
            desired_pipe_angle = PIPE_NEAR_POSITIVE_MAX_DEG;
        } else if (desired_pipe_angle < -PIPE_NEAR_NEGATIVE_MAX_DEG) {
            desired_pipe_angle = -PIPE_NEAR_NEGATIVE_MAX_DEG;
        }
    }

    /* 对最终管角继续做斜率限制，降低水管和小球惯性造成的过冲。 */
    change = desired_pipe_angle - pipe_angle;
    if (change > PIPE_ANGLE_SLEW_PER_CYCLE) {
        change = PIPE_ANGLE_SLEW_PER_CYCLE;
    } else if (change < -PIPE_ANGLE_SLEW_PER_CYCLE) {
        change = -PIPE_ANGLE_SLEW_PER_CYCLE;
    }
    pipe_angle += change;
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
    OLED_Refresh();
    g_oled_refresh_page = 8U;
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
    delay_ms(500U);
    Emm_V5_En_Control(EMM_ADDRESS, true, false);
    delay_ms(10U);
    /* MCU复位后先终止驱动器可能残留的旧运动，再采集本次启动的机械零角。 */
    Emm_V5_Stop_Now(EMM_ADDRESS, false);
    delay_ms(10U);

    while (1) {
        uint8_t received;
        uint8_t control_executed = 0U;
        uint32_t now;

        LLM_UART_Poll();
        while (UART_read_received_byte(&received) != 0U) {
            (void)Sensor_PushByte(received);
        }
        now = g_milliseconds;
        Motor_ControlService(now);
        /* 每1 ms读取一次旋钮，使20次按钮消抖计数对应真实20 ms。 */
        if ((uint32_t)(now - g_last_encoder_ms) >= 1U) {
            g_last_encoder_ms = now;
            Target_PositionUpdate();
        }

        if ((uint32_t)(now - g_last_control_ms) >= CONTROL_PERIOD_MS) {
            g_last_control_ms += CONTROL_PERIOD_MS;
            control_executed = 1U;

            if ((g_position_valid != 0U) &&
                ((uint32_t)(now - g_last_frame_ms) <= SENSOR_TIMEOUT_MS)) {
                Ball_ControlUpdate();
            } else {
                Ball_ControlStop();
            }

        }

        if ((uint32_t)(now - g_last_oled_ms) >= OLED_UPDATE_PERIOD_MS) {
            g_last_oled_ms = now;
            OLED_ShowControlInfo();
        }

        if ((control_executed != 0U) && (g_oled_refresh_page < 8U)) {
            OLED_RefreshPage(g_oled_refresh_page++);
        }
    }
}
