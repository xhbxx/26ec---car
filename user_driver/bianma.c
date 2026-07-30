#include "bianma.h"

static uint8_t g_last_ab = 0U;
static int8_t g_rotation_accumulator = 0;
static uint8_t g_button_state = 1U;
static uint8_t g_button_last_raw = 1U;
static uint8_t g_button_debounce_count = 0U;
static uint8_t g_pid_selection = 0U;

/**
 * GPIO已经由SYSCFG_DL_init()配置，本函数只记录初始电平。
 * 不能在这里复位GPIOA/GPIOB，否则会破坏UART和OLED引脚配置。
 */
void Bianma_Init(void)
{
    uint8_t a = (DL_GPIO_readPins(
        Bianma_PORT, Bianma_ENC_A_PIN) != 0U) ? 1U : 0U;
    uint8_t b = (DL_GPIO_readPins(
        Bianma_PORT, Bianma_ENC_B_PIN) != 0U) ? 1U : 0U;

    g_last_ab = (uint8_t)((a << 1U) | b);
    g_rotation_accumulator = 0;
    g_button_last_raw = (DL_GPIO_readPins(
        Bianma_PORT, Bianma_ENC_SW_PIN) != 0U) ? 1U : 0U;
    g_button_state = g_button_last_raw;
    g_button_debounce_count = 0U;
}

/**
 * 使用完整四状态表解码A/B相。
 * 四个合法跳变累计为旋钮的一格，可过滤机械触点抖动和非法同时跳变。
 */
int8_t Bianma_GetRotation(void)
{
    static const int8_t transition_table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
    uint8_t a = (DL_GPIO_readPins(
        Bianma_PORT, Bianma_ENC_A_PIN) != 0U) ? 1U : 0U;
    uint8_t b = (DL_GPIO_readPins(
        Bianma_PORT, Bianma_ENC_B_PIN) != 0U) ? 1U : 0U;
    uint8_t current_ab = (uint8_t)((a << 1U) | b);
    uint8_t transition = (uint8_t)((g_last_ab << 2U) | current_ab);

    g_last_ab = current_ab;
    g_rotation_accumulator += transition_table[transition];

    if (g_rotation_accumulator >= 4) {
        g_rotation_accumulator = 0;
        return 1;
    }
    if (g_rotation_accumulator <= -4) {
        g_rotation_accumulator = 0;
        return -1;
    }
    return 0;
}

/** 每1ms轮询一次，使用20ms稳定时间生成单次按下事件。 */
uint8_t Bianma_Button_Pressed(void)
{
    uint8_t raw = (DL_GPIO_readPins(
        Bianma_PORT, Bianma_ENC_SW_PIN) != 0U) ? 1U : 0U;

    if (raw != g_button_last_raw) {
        g_button_last_raw = raw;
        g_button_debounce_count = 20U;
        return 0U;
    }

    if (g_button_debounce_count > 0U) {
        g_button_debounce_count--;
        if ((g_button_debounce_count == 0U) && (raw != g_button_state)) {
            g_button_state = raw;
            return (raw == 0U) ? 1U : 0U;
        }
    }
    return 0U;
}

/**
 * 旋钮SW使用内部上拉，按下时应将PB19接地，因此低电平表示按下。
 * 此函数用于启动按键的即时判定和OLED接线诊断。
 */
uint8_t Bianma_Button_IsPressed(void)
{
    return (DL_GPIO_readPins(Bianma_PORT, Bianma_ENC_SW_PIN) == 0U)
        ? 1U : 0U;
}

uint8_t Bianma_GetParamIndex(void)
{
    return g_pid_selection;
}

/** 兼容原来的P/I/D旋钮调节接口。 */
void Bianma_ProcessEncoder(int *p, int *i, int *d)
{
    int8_t rotation;

    if ((p == 0) || (i == 0) || (d == 0)) {
        return;
    }
    if (Bianma_Button_Pressed() != 0U) {
        g_pid_selection = (uint8_t)((g_pid_selection + 1U) % 3U);
    }

    rotation = Bianma_GetRotation();
    if (g_pid_selection == 0U) {
        *p += rotation;
    } else if (g_pid_selection == 1U) {
        *i += rotation;
    } else {
        *d += rotation;
    }
}
