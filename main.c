#include "ti_msp_dl_config.h"

#include "Emm_V5.h"
#include "bianma.h"
#include "delay.h"
#include "main_program_select.h"
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
#define SPEED_SAMPLE_STALE_MS          (200UL)
#define OLED_UPDATE_PERIOD_MS          (245UL)
#define MOTOR_FEEDBACK_QUERY_MS        (40UL)
#define MOTOR_STARTUP_QUERY_MS         (20UL)
#define MOTOR_COMMAND_PERIOD_MS        (40UL)
#define MOTOR_FEEDBACK_TIMEOUT_MS      (200UL)
/* 查询实时位置后最多等待15ms；该期间不能再发送位置控制命令。 */
#define MOTOR_FEEDBACK_RESPONSE_WAIT_MS (15UL)
#define MOTOR_ENABLE_SETTLE_MS         (120U)
#define MOTOR_STOP_SETTLE_MS           (50U)
/* 上电后主动等待首帧电机位置反馈的最长时间；仅初始化阶段使用，不影响20ms控制周期。 */
#define MOTOR_STARTUP_SYNC_TIMEOUT_MS  (1000UL)
/* PID只允许小角度运动，反馈相对启动零点超出该范围时视为异常帧。 */
#define MOTOR_FEEDBACK_ANGLE_LIMIT_DEG (20.0f)

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
#define POSITION_DEFAULT_TARGET        (245U)
/* 旋钮每转动一格，目标位置增加或减少的数值。 */
#define POSITION_TARGET_STEP           (1)

/* Mod4/Mod5：小车全程保持同方向恒定加速度，不设置减速补偿段。 */
#define MODE4_MOTION_COMPENSATION_DEG  (2.40f)
/* Mod5 与Mod4使用相同的补偿角和触发逻辑。 */
#define MODE5_MOTION_COMPENSATION_DEG  (2.40f)
/* 检测到第一次运动后延迟2秒，再开始输出车辆补偿角。 */
#define MODE_COMP_DELAY_MS              (2000UL)
/* Mod4/Mod5只有球偏离默认位置超过该值时才启动位置回正。 */
#define MODE_COMP_RETURN_ERROR_LIMIT   (100.0f)

/*
 * 编码器按键测试顺序：球手动放在C≈250后，按下编码器，先向360运动；到达360±5后不停车，立即反向前往125。
 * 允许惯性越过360，但370开始强制回拉，375为硬保护线。
 */
/* 历史起点窗口参数；当前按键启动不检查此窗口，仍保留以兼容原工程。 */
#define SEQUENCE_START_MIN             (240U)
#define SEQUENCE_START_MAX             (250U)
/* 第1段的视觉坐标目标：按键后小球从约250向坐标增大方向运动到360。 */
#define SEQUENCE_FIRST_TARGET          (130U)
/* 第2段及最终保持的视觉坐标目标：到360后立即反向，最终稳定在125。 */
#define SEQUENCE_FINAL_TARGET          (350U)
/*
 * 第1段到达容差，单位为视觉坐标值。小球向360单向运动时，C>=355便切换回程；
 * 这样即使某一帧从354直接跳到366，也不会漏掉换向，实际允许范围为360±5。
 */
#define SEQUENCE_FIRST_TOLERANCE       (22U)
/* 历史“起点稳定帧数”参数；当前按键启动流程不使用，保留但不参与判断。 */
#define SEQUENCE_START_STABLE_CYCLES   (5U)
/* 在125±5内且低速时，必须连续满足5个新的视觉位置帧才确认最终保持。 */
#define SEQUENCE_FINAL_STABLE_CYCLES   (5U)
/* 最终保持的位置容差，单位为视觉坐标值，允许范围为120~130。 */
#define SEQUENCE_FINAL_TOLERANCE       (7U)
/* 返回337阶段若因惯性仍越过370，开始强制限制为负方向回拉角。 */
#define SEQUENCE_GUARD_POSITION        (120U)
/* 返回125阶段越过375时，跳过普通PID斜率限制并直接下发硬回拉角。 */
#define SEQUENCE_HARD_LIMIT_POSITION   (115U)
/* C>=370时允许的最大管角，负号表示朝坐标减小方向拉回。单位：度。 */
#define SEQUENCE_GUARD_ANGLE_DEG       (4.0f)
/* C>=375时直接使用的安全回拉管角，力度大于普通保护角。单位：度。 */
#define SEQUENCE_HARD_GUARD_ANGLE_DEG  (5.0f)
/* 360后反向时每20ms允许的最大管角变化量，帮助快速撤销原正向推力。单位：度/20ms。 */
#define SEQUENCE_REVERSE_SLEW_DEG      (0.55f)
/* 位置死区：|目标位置-当前位置|不超过该值时，目标速度置零，防止目标附近来回动作。 */
/* 只在非常接近目标时锁定；误差超过2立即恢复缓慢调整。 */
#define POSITION_DEADBAND              (8.0f)
#define POSITION_DEADBAND_EXIT         (8.0f)
/* 在位置死区内且|CS|小于该值时，直接保持上电记录的水平零角，不再继续PID微调。 */
#define DEADBAND_LEVEL_STOP_SPEED      (10.0f)
/* 只有球速足够低并进入±1时才判定到达，防止高速穿过目标时过早停止控制。 */
#define POSITION_LOCK_SPEED            (1.95f)
/* 实测静止CS为0，只保留很小的速度噪声区，避免把真实慢速微调误判为静止。 */
#define SPEED_ZERO_DEADBAND             (3.0f)
/*
 * 位置PID只反馈实际位置；速度阻尼作为独立修正项叠加到外环输出。
 * 原预测反馈等效阻尼约为POSITION_KP*0.1=0.03，当前只保留很小的0.008阻尼。
 */
#define POSITION_SPEED_DAMPING_GAIN    (0.020f)
/*
 * 远距离控制：运动时适度放大PID角度；静止时保证管角超过实测静摩擦阈值。
 * 最低角只在球静止时使用，检测到运动后立即恢复速度闭环输出。
 */
#define POSITION_FAR_ZONE              (7.0f) /* |T-C|大于50进入强驱动区 */
#define PIPE_FAR_ANGLE_GAIN            (1.7f) /* 远距离运动中放大速度环角度 */
#define PIPE_FAR_POSITIVE_MIN_DEG      (2.50f) /* 远距离静止时正方向最低启动角 */
#define PIPE_FAR_NEGATIVE_MIN_DEG      (2.50f) /* 远距离静止时负方向最低启动角 */
/* 未到目标但球已静止、target_speed又接近0时使用的小恢复速度。 */
#define POSITION_RECOVERY_MIN_SPEED    (30.0f)
/* 每20ms允许目标速度改变的最大值，避免进入死区时从PID输出硬切到0。 */
#define TARGET_SPEED_SLEW_PER_CYCLE    (4.0f)
/*
 * 制动时允许目标速度更快地减小或反向。
 * 这不是提高正常追踪速度，而是缩短“球已经有惯性、控制器却还在慢慢撤销旧速度”的时间。
 */
#define TARGET_SPEED_BRAKE_SLEW        (7.6f)

/*
 * 位置外环 PID：输入目标位置和当前位置，输出 target_speed。
 * 调参时先令 POSITION_KI=0，调整 KP、KD，最后再少量增加 KI。
 */
/* 位置比例系数：误差越大，要求的目标速度越大；过大会过冲和来回摆动，过小则响应慢。 */
#define POSITION_KP                    (2.1f)
/* 位置积分系数：消除长期位置偏差；过大会积分累积并导致明显过冲，通常只使用很小数值。 */
#define POSITION_KI                    (0.03f)
/* 位置微分系数：根据误差变化提前减速、增加阻尼；过大会放大位置噪声并造成电机抖动。 */
#define POSITION_KD                    (0.003f)
/* 位置外环最大输出，即 target_speed 的绝对值上限；越大允许7小球移动得越快，也越容易冲过目标。 */
#define POSITION_MAX_SPEED             (70.0f)
/* 位置环积分累计上限，用于防止长时间大误差造成积分饱和；不是速度或角度上限。 */
#define POSITION_INTEGRAL_LIMIT        (400.0f)

/*
 * 速度内环 PID：输入 target_speed 和滤波后的 current_speed，输出 pipe_angle。
 * 它决定水管需要倾斜多少来使小球速度跟随位置外环的要求。
 */
/* 速度比例系数：速度误差对应的即时倾角；增大可提高动作幅度，过大会造成速度震荡。 */
#define SPEED_KP                       (0.07f)
/* 速度积分系数：补偿摩擦、坡度等造成的长期速度不足；过大会持续加大倾角并导致过冲。 */
#define SPEED_KI                       (0.01f)
/* 速度微分系数：抑制速度突然变化；速度反馈噪声较大，因此通常只能使用很小数值。 */
#define SPEED_KD                       (0.01f)
/* 速度内环最终输出的机械管角上限，单位为度；同时限制正、负两个方向为 ±该数值。 */
#define PIPE_MAX_ANGLE_DEG             (15.0f)
/*
 * 负管角方向的机构力度补偿：1.0表示不补偿，数值越大，靠近500一侧的回拉幅度越大。
 * 补偿后的角度仍会被 PIPE_MAX_ANGLE_DEG 限制，不会突破机械角度上限。
 */
#define PIPE_NEGATIVE_ANGLE_GAIN       (1.00f)
/* 往0方向使用负管角，单独限制该方向的最大幅度，避免下降方向动作过大。 */
#define PIPE_NEGATIVE_MAX_ANGLE_DEG    (15.0f)
/* 靠近目标时分方向限制管角；正方向需要更强制动力，负方向保持原限制。 */
#define PIPE_NEAR_ZONE                 (22.0f)
#define PIPE_NEAR_POSITIVE_MAX_DEG     (1.8f)
#define PIPE_NEAR_NEGATIVE_MAX_DEG     (1.7f)
/*
 * 实测静摩擦不对称：往500方向约1°才能可靠启动，新的零点下往0方向需超过1.02°。
 * 两个数值只在球静止且仍在目标外时使用，运动后立即交回速度PID。
 */
#define PIPE_RESTART_POSITIVE_DEG      (1.50f)
#define PIPE_RESTART_NEGATIVE_DEG      (1.50f)
/* 每20ms允许管角目标改变的最大角度，限制电机和水管的瞬时动作。 */
#define PIPE_ANGLE_SLEW_PER_CYCLE      (0.53f)
/* 减小旧角度或反向制动时允许更快变化，避免水管几百毫秒后才建立制动力。 */
#define PIPE_ANGLE_BRAKE_SLEW          (0.60f)
/* 死区内速度低于该值时认为小球基本静止，管角平滑回到机械零角。 */
#define BALL_STOP_SPEED_THRESHOLD      (10.0f)
/* 死区内每周期保留的积分比例，逐渐卸掉旧方向积分而不是瞬间清零。 */
#define PID_INTEGRAL_DECAY             (0.0f)
/* 速度环积分累计上限，防止小球长期不动时积分不断增加并突然产生过大倾角。 */
#define SPEED_INTEGRAL_LIMIT           (180.0f)
/* 速度一阶低通系数：越大越跟手但噪声越明显，越小越平滑但速度反馈延迟越大，范围0~1。 */
#define SPEED_FILTER_ALPHA             (0.38f)
/* 两个 PID 共用的微分低通系数：越大越灵敏，越小越平滑，范围0~1。 */
#define PID_DERIVATIVE_ALPHA           (0.3f)

#define EMM_ADDRESS                    (1U)
#define EMM_MOVE_SPEED_RPM             (30U)
#define EMM_MOVE_ACCELERATION          (10U)
#define EMM_MIN_COMMAND_ANGLE_DEG      (0.12f)
/* 电机正负管角对应的方向；若机构安装方向变化，应成对交换0U和1U。 */
#define EMM_DIRECTION_POSITIVE         (1U)
#define EMM_DIRECTION_NEGATIVE         (0U)

#define FRAME_SIZE                     (7U)
#define FRAME_SOF_1                    (0xAAU)
#define FRAME_SOF_2                    (0x55U)
#define FRAME_VERSION                  (0x01U)

typedef enum
{
    BALL_SEQUENCE_WAIT_START = 0,
    BALL_SEQUENCE_TO_360,
    BALL_SEQUENCE_TO_125,
    BALL_SEQUENCE_HOLD_125
} BallSequenceState;

/** 新主程序中的三种工作模式，数值与OLED菜单编号保持一致。 */
typedef enum
{
    NEW_MODE_HOLD_250 = 1,
    NEW_MODE_CURRENT_SEQUENCE = 2,
    NEW_MODE_ENCODER_TARGET = 3,
    NEW_MODE_CAR_COMP_1 = 4,
    NEW_MODE_CAR_COMP_2 = 5
} NewOperatingMode;

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
/* 只有连续两帧形成可靠差分后才置1；超时后立即清零并重新建立采样基准。 */
static uint8_t g_speed_valid = 0U;
static float g_motor_requested_angle = 0.0f;
static float g_motor_actual_angle = 0.0f;
static float g_motor_zero_absolute_angle = 0.0f;
static uint32_t g_last_motor_feedback_ms = 0U;
static uint32_t g_last_motor_query_ms = 0U;
static uint32_t g_last_motor_command_ms = 0U;
static uint32_t g_motor_query_sent_ms = 0U;
static uint8_t g_motor_feedback_valid = 0U;
static uint8_t g_motor_feedback_query_pending = 0U;
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
static BallSequenceState g_sequence_state = BALL_SEQUENCE_WAIT_START;
static uint8_t g_sequence_stable_cycles = 0U;
static uint32_t g_sequence_last_frame_count = 0U;
/* 旋钮按键产生一次启动请求，由20 ms状态机消费。 */
static uint8_t g_sequence_start_requested = 0U;

/* 新主程序菜单状态；旧主程序Current_Main不读取这些变量。 */
static NewOperatingMode g_new_selected_mode = NEW_MODE_HOLD_250;
static uint32_t g_car_comp_start_ms = 0U;
static uint8_t g_car_comp_motion_started = 0U;
static int8_t g_car_comp_direction = 0;
static uint8_t g_new_mode_confirmed = 0U;
static uint8_t g_new_mode_input_ready = 0U;
static uint8_t g_new_target_confirmed = 0U;
static uint16_t g_new_encoder_target = POSITION_DEFAULT_TARGET;

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
        g_speed_valid = 0U;
    } else {
        uint32_t elapsed_ms = (uint32_t)(now - g_last_position_sample_ms);

        if ((elapsed_ms > 0U) && (elapsed_ms <= SENSOR_TIMEOUT_MS)) {
            float raw_speed = ((float)position - (float)g_last_position) *
                1000.0f / (float)elapsed_ms;
            current_speed += SPEED_FILTER_ALPHA *
                (raw_speed - current_speed);
            g_last_position = position;
            g_last_position_sample_ms = now;
            g_speed_valid = 1U;
        } else if (elapsed_ms > SENSOR_TIMEOUT_MS) {
            /* 间隔过长时旧速度已无意义，从当前帧重新建立差分基准。 */
            current_speed = 0.0f;
            g_last_position = position;
            g_last_position_sample_ms = now;
            g_speed_valid = 0U;
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
    /* 自动流程不使用旋钮旋转调目标，只读取一次以清空累计旋转事件。 */
    (void)Bianma_GetRotation();

    if ((g_sequence_state == BALL_SEQUENCE_WAIT_START) &&
        ((Bianma_Button_Pressed() != 0U) ||
         (Bianma_Button_IsPressed() != 0U))) {
        g_sequence_start_requested = 1U;
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
    /* 已收到本次查询的完整有效位置帧，允许之后再发送运动命令。 */
    g_motor_feedback_query_pending = 0U;
}

/**
 * 轮询PA31电机反馈、周期查询实时位置，并按实际角度发送绝对位置目标。
 * 绝对目标不会像连续相对增量那样因执行延迟而重复累加。
 */
static void Motor_ControlService(uint32_t now)
{
    uint8_t received;
    uint32_t query_period_ms;

    while (Motor_UART_ReadByte(&received) != 0U) {
        Motor_PushFeedbackByte(received);
    }

    if ((g_motor_feedback_valid != 0U) &&
        ((uint32_t)(now - g_last_motor_feedback_ms) >
         MOTOR_FEEDBACK_TIMEOUT_MS)) {
        g_motor_feedback_valid = 0U;
    }

    /*
     * 查询命令与位置控制命令共用UART0。查询发出后必须先等回复或超时，
     * 否则电机的8字节位置回复会与紧接着发送的新命令交叉，造成M偶发丢失。
     */
    if (g_motor_feedback_query_pending != 0U) {
        if ((uint32_t)(now - g_motor_query_sent_ms) <
            MOTOR_FEEDBACK_RESPONSE_WAIT_MS) {
            return;
        }
        /* 本次查询没有有效回复，解除等待，下一次周期可继续查询或发控制命令。 */
        g_motor_feedback_query_pending = 0U;
    }

    /* 首帧尚未取得时更快重试；拿到机械零点后恢复40ms正常查询周期。 */
    query_period_ms = (g_motor_zero_captured == 0U)
        ? MOTOR_STARTUP_QUERY_MS : MOTOR_FEEDBACK_QUERY_MS;
    if (((uint32_t)(now - g_last_motor_query_ms) >= query_period_ms) &&
        ((uint32_t)(now - g_last_motor_command_ms) >= 5U)) {
        g_last_motor_query_ms = now;
        g_motor_query_sent_ms = now;
        g_motor_feedback_query_pending = 1U;
        Emm_V5_Read_Sys_Params(EMM_ADDRESS, S_CPOS);
        return;
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

/**
 * 上电阶段主动同步电机实时位置。
 *
 * 电机驱动器与MSPM0同时上电时，驱动器的内部串口/编码器就绪通常晚于MCU。
 * 若直接进入主循环，第一次查询可能落在驱动器未就绪阶段，OLED上的M需要等待
 * 后续刷新才出现。这里在不发送任何运动角度的前提下，持续查询首帧位置：
 * - 首帧有效位置被Motor_PushFeedbackByte()记录为机械零角；
 * - 成功后立即退出；
 * - 1秒仍无反馈也不死锁，仍会进入原有主循环继续正常查询。
 */
static void Motor_StartupSynchronize(void)
{
    uint32_t start_ms = g_milliseconds;

    /* 清除上电瞬间可能残留的半帧，强制下一帧从Addr开始重新解析。 */
    g_motor_rx_index = 0U;
    g_motor_feedback_valid = 0U;
    g_motor_feedback_query_pending = 0U;
    g_motor_zero_captured = 0U;

    /* 让Motor_ControlService第一次调用立即发出S_CPOS查询，而非等待周期到期。 */
    g_last_motor_query_ms = start_ms - MOTOR_STARTUP_QUERY_MS;
    g_last_motor_command_ms = start_ms - MOTOR_COMMAND_PERIOD_MS;

    while ((g_motor_zero_captured == 0U) &&
           ((uint32_t)(g_milliseconds - start_ms) <
            MOTOR_STARTUP_SYNC_TIMEOUT_MS)) {
        Motor_ControlService(g_milliseconds);
        /* 1ms期间UART0 RX中断仍可接收8字节反馈帧。 */
        delay_ms(1U);
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
        /* 测试固定T时禁止UART SETPOINT修改目标：target_position = value; */
        (void)value;
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

/* 新模式菜单确认时需要立即切换到数据界面，因此提前声明显示函数。 */
static void OLED_ShowControlInfo(void);

/**
 * 新主程序上电菜单：旋转编码器移动“>”，按下确认当前模式。
 * 菜单只更新OLED显存，主循环仍按原来的分页方式刷新，避免长时间阻塞UART。
 */
static void OLED_ShowModeMenu(void)
{
    uint8_t second_page = (g_new_selected_mode >= NEW_MODE_CAR_COMP_1)
        ? 1U : 0U;

    /*
     * 只清显存，不把空白帧立即发送到屏幕。
     * 原OLED_Clear()会先整屏刷黑，连续调用时会闪烁，并长时间阻塞编码器采样。
     */
    OLED_ClearBuffer();
    OLED_ShowString(18U, 0U,
        (u8 *)(second_page != 0U ? "SELECT 2/2" : "SELECT 1/2"), 12U);

    if (second_page != 0U) {
        OLED_ShowString(0U, 16U,
            (u8 *)((g_new_selected_mode == NEW_MODE_CAR_COMP_1) ? ">" : " "),
            12U);
        OLED_ShowString(12U, 16U, (u8 *)"4 CAR COMP A", 12U);

        OLED_ShowString(0U, 32U,
            (u8 *)((g_new_selected_mode == NEW_MODE_CAR_COMP_2) ? ">" : " "),
            12U);
        OLED_ShowString(12U, 32U, (u8 *)"5 CAR COMP B", 12U);
        OLED_ShowString(12U, 48U, (u8 *)"ROTATE TO PAGE", 12U);
        g_oled_refresh_page = 0U;
        return;
    }

    OLED_ShowString(0U, 16U,
        (u8 *)((g_new_selected_mode == NEW_MODE_HOLD_250) ? ">" : " "),
        12U);
    OLED_ShowString(12U, 16U, (u8 *)"1 HOLD 250", 12U);

    OLED_ShowString(0U, 32U,
        (u8 *)((g_new_selected_mode == NEW_MODE_CURRENT_SEQUENCE) ? ">" : " "),
        12U);
    OLED_ShowString(12U, 32U, (u8 *)"2 130 TO 350", 12U);

    OLED_ShowString(0U, 48U,
        (u8 *)((g_new_selected_mode == NEW_MODE_ENCODER_TARGET) ? ">" : " "),
        12U);
    OLED_ShowString(12U, 48U, (u8 *)"3 SET TARGET", 12U);
    g_oled_refresh_page = 0U;
}

/**
 * 进入一个新模式前清除上一模式可能遗留的PID积分、目标速度和顺序状态。
 * 不清除视觉UART和电机位置反馈，因此确认模式后可立即继续使用已有数据。
 */
static void NewMode_ResetControlState(void)
{
    PID_Reset(&g_position_pid);
    PID_Reset(&g_speed_pid);
    target_speed = 0.0f;
    pipe_angle = 0.0f;
    g_position_locked = 0U;
    g_sequence_state = BALL_SEQUENCE_WAIT_START;
    g_sequence_stable_cycles = 0U;
    g_sequence_start_requested = 0U;
    g_car_comp_start_ms = g_milliseconds;
    g_car_comp_motion_started = 0U;
    g_car_comp_direction = 0;
    Motor_SetAngle(0.0f);
}

/**
 * 新主程序的旋钮输入处理，每1ms调用一次。
 *
 * 未选择模式：旋转在1/2/3之间循环，按下确认并切换到数据界面。
 * 模式2：确认模式且松开第一次按键后，再按一次才启动原250->360->125流程。
 * 模式3：旋转以5为步长选择0~500目标，按下后锁定目标并开始闭环控制。
 */
static void NewMode_ProcessEncoder(int8_t rotation, uint8_t poll_button)
{
    /* A/B由主循环高频采样；SW仍按1ms周期采样，保持20ms按键消抖时间。 */
    uint8_t pressed = (poll_button != 0U) ? Bianma_Button_Pressed() : 0U;

    if (g_new_mode_confirmed == 0U) {
        if (rotation > 0) {
            g_new_selected_mode = (g_new_selected_mode >=
                NEW_MODE_CAR_COMP_2) ? NEW_MODE_HOLD_250 :
                (NewOperatingMode)((uint8_t)g_new_selected_mode + 1U);
            OLED_ShowModeMenu();
        } else if (rotation < 0) {
            g_new_selected_mode = (g_new_selected_mode <=
                NEW_MODE_HOLD_250) ? NEW_MODE_CAR_COMP_2 :
                (NewOperatingMode)((uint8_t)g_new_selected_mode - 1U);
            OLED_ShowModeMenu();
        }

        if (pressed != 0U) {
            g_new_mode_confirmed = 1U;
            /* 必须先等本次选择按键松开，避免同一次按下又触发模式2启动或模式3确认。 */
            g_new_mode_input_ready = 0U;
            g_new_target_confirmed = 0U;
            g_new_encoder_target = POSITION_DEFAULT_TARGET;
            target_position = POSITION_DEFAULT_TARGET;
            NewMode_ResetControlState();
            OLED_ClearBuffer();
            OLED_ShowControlInfo();
            OLED_Refresh();
            g_oled_refresh_page = 8U;
        }
        return;
    }

    if (g_new_mode_input_ready == 0U) {
        /* 模式选择按键完全松开后才允许接收下一次确认事件。 */
        if (Bianma_Button_IsPressed() == 0U) {
            g_new_mode_input_ready = 1U;
        }
        return;
    }

    if (g_new_selected_mode == NEW_MODE_CURRENT_SEQUENCE) {
        /* 模式2完整复用原状态机，只将启动动作改为消抖后的单次按下事件。 */
        (void)rotation;
        if ((g_sequence_state == BALL_SEQUENCE_WAIT_START) &&
            (pressed != 0U)) {
            g_sequence_start_requested = 1U;
        }
    } else if (g_new_selected_mode == NEW_MODE_ENCODER_TARGET) {
        if (g_new_target_confirmed == 0U) {
            int32_t next_target = (int32_t)g_new_encoder_target +
                (int32_t)rotation * 5;

            if (next_target < (int32_t)POSITION_MIN) {
                next_target = (int32_t)POSITION_MIN;
            } else if (next_target > (int32_t)POSITION_MAX) {
                next_target = (int32_t)POSITION_MAX;
            }
            g_new_encoder_target = (uint16_t)next_target;
            target_position = g_new_encoder_target;

            if (pressed != 0U) {
                /* 目标确认后保持该T，不再允许旋钮误触改变运行中的目标。 */
                g_new_target_confirmed = 1U;
                PID_Reset(&g_position_pid);
                PID_Reset(&g_speed_pid);
                target_speed = 0.0f;
                pipe_angle = 0.0f;
                g_position_locked = 0U;
            }
        }
    } else {
        /* 模式1目标固定250，运行后不再使用旋钮改变目标。 */
        (void)rotation;
        (void)pressed;
    }
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
    OLED_ShowString(84U, 0U, (u8 *)"S:", 12U);
    OLED_ShowNum(96U, 0U, (uint32_t)g_sequence_state, 1U, 12U);
    OLED_ShowString(108U, 0U, (u8 *)"K:", 12U);
    OLED_ShowNum(120U, 0U, (uint32_t)Bianma_Button_IsPressed(), 1U, 12U);

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
    g_speed_valid = 0U;
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
 * 自动目标状态机，只负责切换现有双环PID的target_position，不另建控制链路。
 * S0等待编码器按键，T固定为250且水管归零；S1前往360；S2立即反向前往125；S3在125±5内保持。
 */
/** 等待按键时保持T=250和机械零角，但不清掉位置反馈或UART接收状态。 */
static void Ball_ControlIdle(void)
{
    PID_Reset(&g_position_pid);
    PID_Reset(&g_speed_pid);
    target_speed = 0.0f;
    pipe_angle = 0.0f;
    g_position_locked = 0U;
    Motor_SetAngle(0.0f);
}

static void Ball_SequenceUpdate(void)
{
    float speed_abs = (current_speed < 0.0f)
        ? -current_speed : current_speed;
    uint8_t new_position_sample =
        (valid_frame_count != g_sequence_last_frame_count) ? 1U : 0U;

    if (new_position_sample != 0U) {
        g_sequence_last_frame_count = valid_frame_count;
    }

    switch (g_sequence_state) {
    case BALL_SEQUENCE_WAIT_START:
        /*
         * 等待阶段：OLED显示T=250，但Ball_ControlIdle()会让水管保持零角，
         * 因此用户可手动把球放到C≈250，未按键前不会被PID主动推动。
         */
        target_position = POSITION_DEFAULT_TARGET;
        if (g_sequence_start_requested != 0U) {
            /* 编码器按键已经消抖确认：只清本次运动残留的PID状态，再开始第1段。 */
            g_sequence_stable_cycles = 0U;
            g_sequence_start_requested = 0U;
            g_sequence_state = BALL_SEQUENCE_TO_360;
            target_position = SEQUENCE_FIRST_TARGET;
            PID_Reset(&g_position_pid);
            PID_Reset(&g_speed_pid);
            g_position_locked = 0U;
        }
        break;

    case BALL_SEQUENCE_TO_360:
        target_position = SEQUENCE_FIRST_TARGET;
        /*
         * 向360运动时进入360-5即可认为到达。由于本阶段只会向增大方向运动，
         * 使用下边界触发可避免视觉单帧跳过360+5时错过换向机会。
         */
        if (current_position <=
            (SEQUENCE_FIRST_TARGET + SEQUENCE_FIRST_TOLERANCE)) {
            /* 到达360±5范围后直接反向；清除上一阶段的正向积分和目标速度。 */
            g_sequence_state = BALL_SEQUENCE_TO_125;
            target_position = SEQUENCE_FINAL_TARGET;
            target_speed = 0.0f;
            PID_Reset(&g_position_pid);
            PID_Reset(&g_speed_pid);
            g_position_locked = 0U;
        }
        break;

    case BALL_SEQUENCE_TO_125:
        target_position = SEQUENCE_FINAL_TARGET;
        /*
         * 只有收到新视觉帧、位置落在120~130、且|CS|不大于POSITION_LOCK_SPEED时才计一次稳定帧。
         * 这避免小球高速穿过125时被误判为结束；中途任何一帧不满足都会重新计数。
         */
        if ((new_position_sample != 0U) &&
            (current_position >=
             (SEQUENCE_FINAL_TARGET - SEQUENCE_FINAL_TOLERANCE)) &&
            (current_position <=
             (SEQUENCE_FINAL_TARGET + SEQUENCE_FINAL_TOLERANCE)) &&
            (speed_abs <= POSITION_LOCK_SPEED)) {
            if (++g_sequence_stable_cycles >=
                SEQUENCE_FINAL_STABLE_CYCLES) {
                g_sequence_stable_cycles = 0U;
                g_sequence_state = BALL_SEQUENCE_HOLD_125;
            }
        } else if (new_position_sample != 0U) {
            g_sequence_stable_cycles = 0U;
        }
        break;

    case BALL_SEQUENCE_HOLD_125:
    default:
        /* 最终阶段始终保持T=125；仍运行原双环PID，受扰动离开125后会自动微调回来。 */
        target_position = SEQUENCE_FINAL_TARGET;
        break;
    }
}

/* Mod4/Mod5的补偿保持：不让位置环把球主动拉回默认位置，
 * 只输出运行补偿角，并在补偿结束后把水管缓慢回到水平零角。 */
static void Ball_CarCompensationOnlyUpdate(void)
{
    float desired_angle = 0.0f;
    float change;
    float compensation_angle;

    target_speed = 0.0f;
    PID_Reset(&g_position_pid);
    PID_Reset(&g_speed_pid);

    if ((g_car_comp_motion_started == 0U) &&
        (g_speed_valid != 0U) &&
        (current_speed > SPEED_ZERO_DEADBAND ||
         current_speed < -SPEED_ZERO_DEADBAND)) {
        g_car_comp_motion_started = 1U;
        g_car_comp_start_ms = g_milliseconds;
        g_car_comp_direction = (current_speed > 0.0f) ? 1 : -1;
    }
    if ((g_car_comp_motion_started != 0U) &&
        ((uint32_t)(g_milliseconds - g_car_comp_start_ms) >=
         MODE_COMP_DELAY_MS)) {
        /* 延迟结束后直接输出完整角度，并一直保持。 */
        compensation_angle =
            (g_new_selected_mode == NEW_MODE_CAR_COMP_1)
            ? MODE4_MOTION_COMPENSATION_DEG
            : MODE5_MOTION_COMPENSATION_DEG;
        desired_angle = compensation_angle * (float)g_car_comp_direction;
    }

    change = desired_angle - pipe_angle;
    if (change > PIPE_ANGLE_SLEW_PER_CYCLE) {
        change = PIPE_ANGLE_SLEW_PER_CYCLE;
    } else if (change < -PIPE_ANGLE_SLEW_PER_CYCLE) {
        change = -PIPE_ANGLE_SLEW_PER_CYCLE;
    }
    pipe_angle += change;
    Motor_SetAngle(pipe_angle);
}

/** 固定20 ms执行位置外环和速度内环，并将目标管角交给已有电机驱动。 */
static void Ball_ControlUpdate(void)
{
    float position_error = (float)target_position -
        (float)current_position;
    float desired_target_speed;
    float desired_pipe_angle;
    float change;
    float absolute_position_error;
    float absolute_current_speed;
    float control_speed;
    float target_speed_slew;
    float pipe_angle_slew;
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
        g_speed_valid = 0U;
        /* 下一帧只用于重建位置/时间基准，不能跨越丢帧区间直接计算速度。 */
        g_speed_sample_initialized = 0U;
    }

    /*
     * 速度反馈无效时不能把0当成真实速度继续运行速度环。但位置帧仍然有效时，
     * 若小球还在目标外，保持旧管角会造成“未到T却永远不再尝试移动”。
     * 因此这里只按位置误差给一个很小的开环试探角；不使用远距离强驱动角，
     * 等连续两帧恢复出可靠速度后，再自动交回原双环PID控制。
     */
    if (g_speed_valid == 0U) {
        float probe_angle = 0.0f;

        PID_DecayIntegral(&g_position_pid, PID_INTEGRAL_DECAY);
        PID_DecayIntegral(&g_speed_pid, PID_INTEGRAL_DECAY);
        target_speed = 0.0f;

        if (position_error > POSITION_DEADBAND_EXIT) {
            probe_angle = PIPE_RESTART_POSITIVE_DEG;
        } else if (position_error < -POSITION_DEADBAND_EXIT) {
            probe_angle = -PIPE_RESTART_NEGATIVE_DEG;
        }

        /* 试探角同样经过斜率限制，避免速度样本有效/无效切换时跳变。 */
        change = probe_angle - pipe_angle;
        if (change > PIPE_ANGLE_SLEW_PER_CYCLE) {
            change = PIPE_ANGLE_SLEW_PER_CYCLE;
        } else if (change < -PIPE_ANGLE_SLEW_PER_CYCLE) {
            change = -PIPE_ANGLE_SLEW_PER_CYCLE;
        }
        pipe_angle += change;
        Motor_SetAngle(pipe_angle);
        return;
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
    /*
     * 死区和“最终锁定”分开判断：
     * 只要位置进入死区，就立即要求目标速度为0，让速度环根据仍存在的CS主动反向制动；
     * g_position_locked仍要求低速，用于确认小球已经真正停稳，而不是高速穿过目标。
     */
    in_deadband = (uint8_t)(
        (absolute_position_error <= POSITION_DEADBAND) ||
        ((g_position_locked != 0U) &&
         (absolute_position_error <= POSITION_DEADBAND_EXIT)));

    /*
     * 第一阶段必须实际读到360才换向，不能让通用的+-1位置死区在359提前锁定。
     * 该例外只用于S1；返回125及最终保持仍完整沿用原死区和低速判定。
     */
    if ((g_sequence_state == BALL_SEQUENCE_TO_360) &&
        (current_position > SEQUENCE_FIRST_TARGET)) {
        g_position_locked = 0U;
        in_deadband = 0U;
    }

    /*
     * 写死停止条件：位置已经进入设定死区，并且小球绝对速度小于10时，
     * 立即清除两环历史输出，将目标速度和管角同时置零。
     * Motor_SetAngle(0)使用开机读取的机械零点，因此电机会回到并保持水平角度；
     * 若位置随后跑出死区，本函数下一周期会自动恢复原双环PID控制。
     */
    if ((in_deadband != 0U) &&
        (absolute_current_speed < DEADBAND_LEVEL_STOP_SPEED)) {
        target_speed = 0.0f;
        pipe_angle = 0.0f;
        PID_Reset(&g_position_pid);
        PID_Reset(&g_speed_pid);
        Motor_SetAngle(0.0f);
        return;
    }

#if POSITION_LOOP_ENABLED
    desired_target_speed = PID_UpdateConditional(&g_position_pid,
        (float)target_position, (float)current_position,
        (uint8_t)(in_deadband == 0U));
    if (in_deadband != 0U) {
        desired_target_speed = 0.0f;
        PID_DecayIntegral(&g_position_pid, PID_INTEGRAL_DECAY);
    } else {
        /*
         * 速度方向与目标方向相同时减小目标速度，形成提前制动；
         * 球反向远离目标时该项会增强追回速度。它不改变PID的实际位置反馈点。
         */
        desired_target_speed -=
            POSITION_SPEED_DAMPING_GAIN * control_speed;
        if (desired_target_speed > POSITION_MAX_SPEED) {
            desired_target_speed = POSITION_MAX_SPEED;
        } else if (desired_target_speed < -POSITION_MAX_SPEED) {
            desired_target_speed = -POSITION_MAX_SPEED;
        }
    }

#else
    /* 单独调内环时使用固定目标速度，不执行位置PID。 */
    desired_target_speed = SPEED_LOOP_TEST_TARGET;
    (void)position_error;
    in_deadband = 0U;
#endif

    if (in_deadband != 0U) {
        /* 死区定义要求TS为0；CS仍送入速度环，因此小球运动时不会撤掉主动制动力。 */
        target_speed = 0.0f;
    } else {
        /* 死区外限制目标速度变化；减速或反向时使用更快的制动斜率。 */
        target_speed_slew = TARGET_SPEED_SLEW_PER_CYCLE;
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
    }

    /*
     * 预测制动或斜率过渡可能使target_speed暂时停在0附近。
     * 如果实际位置仍在死区外且球已经静止，根据真实位置误差恢复一个小速度，
     * 避免“尚未到达目标，但TS=0后永远不再动作”。
     */
    if ((in_deadband == 0U) &&
        (absolute_current_speed <= BALL_STOP_SPEED_THRESHOLD) &&
        (target_speed > -POSITION_RECOVERY_MIN_SPEED) &&
        (target_speed < POSITION_RECOVERY_MIN_SPEED)) {
        if (position_error > 0.0f) {
            target_speed = POSITION_RECOVERY_MIN_SPEED;
        } else if (position_error < 0.0f) {
            target_speed = -POSITION_RECOVERY_MIN_SPEED;
        }
    }

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

    /* Mod4/Mod5：小车全程恒定加速度时，球一旦开始移动就持续加入
     * 同方向固定补偿；第一次触发采用较小幅度并在250ms内平滑升高，
     * 避免第一帧速度跳变叠加补偿造成角度偏大。 */
    if ((g_new_selected_mode == NEW_MODE_CAR_COMP_1) ||
        (g_new_selected_mode == NEW_MODE_CAR_COMP_2)) {
        float compensation_angle =
            (g_new_selected_mode == NEW_MODE_CAR_COMP_1)
            ? MODE4_MOTION_COMPENSATION_DEG
            : MODE5_MOTION_COMPENSATION_DEG;

        if ((g_car_comp_motion_started == 0U) &&
            (g_speed_valid != 0U) &&
            (absolute_current_speed > SPEED_ZERO_DEADBAND)) {
            g_car_comp_motion_started = 1U;
            g_car_comp_start_ms = g_milliseconds;
            g_car_comp_direction = (control_speed > 0.0f) ? 1 : -1;
        }
        if (g_car_comp_motion_started != 0U &&
            (uint32_t)(g_milliseconds - g_car_comp_start_ms) >=
            MODE_COMP_DELAY_MS) {
            /* 延迟结束后直接叠加完整补偿角，不再渐增。 */
            desired_pipe_angle += compensation_angle *
                (float)g_car_comp_direction;
        }
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
        if ((position_error > 0.0f) &&
            (desired_pipe_angle < PIPE_RESTART_POSITIVE_DEG)) {
            desired_pipe_angle = PIPE_RESTART_POSITIVE_DEG;
        } else if ((position_error < 0.0f) &&
            (desired_pipe_angle > -PIPE_RESTART_NEGATIVE_DEG)) {
            desired_pipe_angle = -PIPE_RESTART_NEGATIVE_DEG;
        }
    }

    /*
     * 误差大于120时提高远距离驱动力。
     * 球运动时只放大PID结果；球静止时再保证最终角度达到可启动的最低值。
     */
    if (absolute_position_error > POSITION_FAR_ZONE) {
        desired_pipe_angle *= PIPE_FAR_ANGLE_GAIN;

        if (absolute_current_speed <= BALL_STOP_SPEED_THRESHOLD) {
            if ((position_error > 0.0f) &&
                (desired_pipe_angle < PIPE_FAR_POSITIVE_MIN_DEG)) {
                desired_pipe_angle = PIPE_FAR_POSITIVE_MIN_DEG;
            } else if ((position_error < 0.0f) &&
                (desired_pipe_angle > -PIPE_FAR_NEGATIVE_MIN_DEG)) {
                desired_pipe_angle = -PIPE_FAR_NEGATIVE_MIN_DEG;
            }
        }

        if (desired_pipe_angle > PIPE_MAX_ANGLE_DEG) {
            desired_pipe_angle = PIPE_MAX_ANGLE_DEG;
        } else if (desired_pipe_angle < -PIPE_NEGATIVE_MAX_ANGLE_DEG) {
            desired_pipe_angle = -PIPE_NEGATIVE_MAX_ANGLE_DEG;
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

    /*
     * 360换向后的独立安全保护。370起要求最大允许回拉角，375处跳过
     * 普通斜率直接给硬保护角；仅在返回125阶段生效，不改变正常PID算法。
     */
    if ((g_sequence_state >= BALL_SEQUENCE_TO_125) &&
        (current_position <= SEQUENCE_GUARD_POSITION) &&
        (desired_pipe_angle < SEQUENCE_GUARD_ANGLE_DEG)) {
        desired_pipe_angle = SEQUENCE_GUARD_ANGLE_DEG;
    }
    if ((g_sequence_state >= BALL_SEQUENCE_TO_125) &&
        (current_position <= SEQUENCE_HARD_LIMIT_POSITION)) {
        pipe_angle = SEQUENCE_HARD_GUARD_ANGLE_DEG;
        Motor_SetAngle(pipe_angle);
        return;
    }

    /*
     * 正常加大倾角仍使用0.15°/周期；撤销旧倾角或反向制动使用0.35°/周期。
     * 这样不会提高正常加速冲击，但能明显缩短越过目标后的制动建立时间。
     */
    pipe_angle_slew = PIPE_ANGLE_SLEW_PER_CYCLE;
    if (((pipe_angle > 0.0f) &&
         (desired_pipe_angle < pipe_angle)) ||
        ((pipe_angle < 0.0f) &&
         (desired_pipe_angle > pipe_angle))) {
        pipe_angle_slew = PIPE_ANGLE_BRAKE_SLEW;
    }
    if ((g_sequence_state == BALL_SEQUENCE_TO_125) &&
        (current_position <= SEQUENCE_FIRST_TARGET) &&
        (pipe_angle < desired_pipe_angle)) {
        /* 360后的反向必须比普通微调更快，避免旧正角继续推动小球。 */
        pipe_angle_slew = SEQUENCE_REVERSE_SLEW_DEG;
    }

    change = desired_pipe_angle - pipe_angle;
    if (change > pipe_angle_slew) {
        change = pipe_angle_slew;
    } else if (change < -pipe_angle_slew) {
        change = -pipe_angle_slew;
    }
    pipe_angle += change;
    Motor_SetAngle(pipe_angle);
}

/**
 * 原主程序入口：完整保留原来的编码器按下后执行250->360->125流程。
 * 是否使用该入口由main_program_select.h统一选择。
 */
int Current_Main(void)
{
    SYSCFG_DL_init();
    LLM_UART_Init();
    Motor_UART_EnableRxInterrupt();
    Bianma_Init();
    /*
     * CAM2位置帧改由UART2 RX中断先存入环形缓冲。
     * 这样后面的OLED初始化和电机上电位置同步期间也不会丢失整帧数据。
     */
    UART_EnableRxInterrupt();

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
    /* 闭环驱动器刚上电时UART和编码器尚未完全稳定，不能马上读取位置。 */
    delay_ms(MOTOR_ENABLE_SETTLE_MS);
    /* MCU复位后先终止驱动器可能残留的旧运动，再采集本次启动的机械零角。 */
    Emm_V5_Stop_Now(EMM_ADDRESS, false);
    delay_ms(MOTOR_STOP_SETTLE_MS);
    /* 先获取电机初始绝对位置作为管角零点，再允许后续PID控制。 */
    Motor_UART_EnableRxInterrupt();
    Motor_StartupSynchronize();

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
            /* 只读取编码器按键：按下后启动250->360->125测试流程。 */
            Target_PositionUpdate();
        }

        if ((uint32_t)(now - g_last_control_ms) >= CONTROL_PERIOD_MS) {
            g_last_control_ms += CONTROL_PERIOD_MS;
            control_executed = 1U;

            if ((g_position_valid != 0U) &&
                ((uint32_t)(now - g_last_frame_ms) <= SENSOR_TIMEOUT_MS)) {
                /* 状态机只切换T，PID和电机控制仍使用原有Ball_ControlUpdate。 */
                Ball_SequenceUpdate();
                if (g_sequence_state == BALL_SEQUENCE_WAIT_START) {
                    /* 按键触发前保持T=250和水管机械零角。 */
                    Ball_ControlIdle();
                } else {
                    Ball_ControlUpdate();
                }
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

/**
 * 新主程序入口：上电先选择模式，确认后继续复用当前工程的通信、PID和电机控制。
 *
 * 模式1：T固定为250，收到有效视觉位置后持续闭环保持；
 * 模式2：保留Current_Main中的250->360->125状态机，选择后需再按一次启动；
 * 模式3：旋钮以5为步长选择T，按下确认后闭环保持在该位置。
 */
int New_Main(void)
{
    SYSCFG_DL_init();
    LLM_UART_Init();
    Motor_UART_EnableRxInterrupt();
    Bianma_Init();
    /* CAM2位置帧使用UART2中断环形缓冲，菜单和OLED刷新期间也不会停止收数。 */
    UART_EnableRxInterrupt();

    PID_Init(&g_position_pid, POSITION_KP, POSITION_KI, POSITION_KD,
        CONTROL_DT_S, POSITION_MAX_SPEED, POSITION_INTEGRAL_LIMIT,
        PID_DERIVATIVE_ALPHA);
    PID_Init(&g_speed_pid, SPEED_KP, SPEED_KI, SPEED_KD,
        CONTROL_DT_S, PIPE_MAX_ANGLE_DEG, SPEED_INTEGRAL_LIMIT,
        PID_DERIVATIVE_ALPHA);

    /* 每次上电都从模式1开始显示，不沿用调试器软复位前的菜单状态。 */
    g_new_selected_mode = NEW_MODE_HOLD_250;
    g_new_mode_confirmed = 0U;
    g_new_mode_input_ready = 0U;
    g_new_target_confirmed = 0U;
    g_new_encoder_target = POSITION_DEFAULT_TARGET;
    target_position = POSITION_DEFAULT_TARGET;
    NewMode_ResetControlState();

    OLED_Init();
    OLED_ShowModeMenu();
    OLED_Refresh();
    g_oled_refresh_page = 8U;
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

    delay_ms(500U);
    Emm_V5_En_Control(EMM_ADDRESS, true, false);
    delay_ms(MOTOR_ENABLE_SETTLE_MS);
    Emm_V5_Stop_Now(EMM_ADDRESS, false);
    delay_ms(MOTOR_STOP_SETTLE_MS);
    /* 读取上电机械位置作为本次运行零点，与原主程序使用同一套电机反馈逻辑。 */
    Motor_UART_EnableRxInterrupt();
    Motor_StartupSynchronize();

    while (1) {
        uint8_t received;
        uint8_t control_executed = 0U;
        uint8_t poll_encoder_button = 0U;
        int8_t encoder_rotation;
        uint32_t now;

        LLM_UART_Poll();
        while (UART_read_received_byte(&received) != 0U) {
            (void)Sensor_PushByte(received);
        }
        now = g_milliseconds;
        Motor_ControlService(now);

        /* 菜单选择、模式2启动和模式3目标确认全部使用同一个1ms消抖入口。 */
        /* A/B每次循环采样，避免OLED通信期间漏相位；SW仍保持1ms采样用于按键消抖。 */
        encoder_rotation = Bianma_GetRotation();
        if ((uint32_t)(now - g_last_encoder_ms) >= 1U) {
            g_last_encoder_ms = now;
            poll_encoder_button = 1U;
        }
        if ((encoder_rotation != 0) || (poll_encoder_button != 0U)) {
            NewMode_ProcessEncoder(encoder_rotation, poll_encoder_button);
        }

        if ((uint32_t)(now - g_last_control_ms) >= CONTROL_PERIOD_MS) {
            g_last_control_ms += CONTROL_PERIOD_MS;
            control_executed = 1U;

            if ((g_position_valid != 0U) &&
                ((uint32_t)(now - g_last_frame_ms) <= SENSOR_TIMEOUT_MS)) {
                if (g_new_mode_confirmed == 0U) {
                    /* 菜单阶段只接收并显示位置，水管保持上电零角。 */
                    target_position = POSITION_DEFAULT_TARGET;
                    Ball_ControlIdle();
                } else if (g_new_selected_mode == NEW_MODE_HOLD_250) {
                    /* 模式1：固定目标250，球被扰动后仍会由原双环PID拉回。 */
                    target_position = POSITION_DEFAULT_TARGET;
                    Ball_ControlUpdate();
                } else if ((g_new_selected_mode == NEW_MODE_CAR_COMP_1) ||
                    (g_new_selected_mode == NEW_MODE_CAR_COMP_2)) {
                    float mode_error;
                    target_position = POSITION_DEFAULT_TARGET;
                    mode_error = (float)target_position -
                        (float)current_position;
                    if (mode_error > MODE_COMP_RETURN_ERROR_LIMIT ||
                        mode_error < -MODE_COMP_RETURN_ERROR_LIMIT) {
                        /* 偏离超过100才允许原位置环主动回正。 */
                        Ball_ControlUpdate();
                    } else {
                        /* 偏离不超过100时只做加速度补偿，并回水平零角。 */
                        Ball_CarCompensationOnlyUpdate();
                    }
                } else if (g_new_selected_mode ==
                    NEW_MODE_CURRENT_SEQUENCE) {
                    /* 模式2：完整调用当前main已有的顺序状态机与控制函数。 */
                    Ball_SequenceUpdate();
                    if (g_sequence_state == BALL_SEQUENCE_WAIT_START) {
                        Ball_ControlIdle();
                    } else {
                        Ball_ControlUpdate();
                    }
                } else {
                    /*
                     * 模式3：确认前只显示旋钮选出的T并保持水管零角；
                     * 按下确认后固定该T，继续使用原双环PID控制小球位置。
                     */
                    target_position = g_new_encoder_target;
                    if (g_new_target_confirmed != 0U) {
                        Ball_ControlUpdate();
                    } else {
                        Ball_ControlIdle();
                    }
                }
            } else {
                /* 无有效视觉位置或超过300ms未更新时沿用原安全停止逻辑。 */
                Ball_ControlStop();
            }
        }

        if ((uint32_t)(now - g_last_oled_ms) >= OLED_UPDATE_PERIOD_MS) {
            g_last_oled_ms = now;
            if (g_new_mode_confirmed != 0U) {
                OLED_ShowControlInfo();
            }
            /* 菜单未确认时内容不变，不再每250ms清屏重画。 */
        }

        if ((control_executed != 0U) && (g_oled_refresh_page < 8U)) {
            OLED_RefreshPage(g_oled_refresh_page++);
        }
    }
}
