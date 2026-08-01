#include "uart.h"

#define UART_RX_BUFFER_SIZE            (512U)
#define MOTOR_UART_RX_BUFFER_SIZE      (32U)
#define VEHICLE_UART_RX_BUFFER_SIZE    (256U)

static volatile uint8_t g_uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t g_uart_rx_head = 0U;
static volatile uint16_t g_uart_rx_tail = 0U;
static volatile uint16_t g_uart_rx_overflow_count = 0U;
static volatile uint8_t g_motor_uart_rx_buffer[MOTOR_UART_RX_BUFFER_SIZE];
static volatile uint8_t g_motor_uart_rx_head = 0U;
static volatile uint8_t g_motor_uart_rx_tail = 0U;
static volatile uint8_t g_vehicle_uart_rx_buffer[VEHICLE_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_vehicle_uart_rx_head = 0U;
static volatile uint16_t g_vehicle_uart_rx_tail = 0U;
static volatile uint16_t g_vehicle_uart_rx_overflow_count = 0U;
static volatile uint16_t g_vehicle_uart_error_count = 0U;

#define VEHICLE_UART_ERROR_INTERRUPTS                                \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                          \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                            \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                           \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                          \
     DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |                       \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, (uint8_t) *str);
        str++;
    }
}

/** 从CAM2 UART2中断环形缓冲读取一个字节；无数据返回0。 */
uint8_t UART_read_received_byte(uint8_t *value)
{
    if (value == 0) {
        return 0U;
    }

    if (g_uart_rx_head == g_uart_rx_tail) {
        return 0U;
    }

    *value = g_uart_rx_buffer[g_uart_rx_tail];
    g_uart_rx_tail = (uint16_t)((g_uart_rx_tail + 1U) %
        UART_RX_BUFFER_SIZE);
    return 1U;
}

/**
 * 开启CAM2 UART2接收中断。
 *
 * CAM2会连续发送7字节位置帧，而电机上电同步和OLED刷新都会暂时占用主循环。
 * 中断只负责把字节存入512字节环形缓冲，协议解析仍在原主循环执行，因此不会
 * 改变AA 55 01 SEQ POS_L POS_H CRC8格式，也不会阻塞20ms控制任务。
 */
void UART_EnableRxInterrupt(void)
{
    g_uart_rx_head = 0U;
    g_uart_rx_tail = 0U;
    g_uart_rx_overflow_count = 0U;

    /* 丢弃使能中断前可能残留的半帧，后续解析器会从新的0xAA重新同步。 */
    while (!DL_UART_Main_isRXFIFOEmpty(PRINT_INST)) {
        (void)DL_UART_Main_receiveData(PRINT_INST);
    }
    DL_UART_Main_setRXFIFOThreshold(
        PRINT_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(PRINT_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(PRINT_INST_INT_IRQN);
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);
}

/** 从Emm_V5电机反馈串口UART0/PA31读取一个字节；无数据返回0。 */
uint8_t Motor_UART_ReadByte(uint8_t *value)
{
    if (value == 0) {
        return 0U;
    }

    if (g_motor_uart_rx_head == g_motor_uart_rx_tail) {
        return 0U;
    }
    *value = g_motor_uart_rx_buffer[g_motor_uart_rx_tail];
    g_motor_uart_rx_tail = (uint8_t)((g_motor_uart_rx_tail + 1U) %
        MOTOR_UART_RX_BUFFER_SIZE);
    return 1U;
}

/**
 * UART0反馈只有8字节，但OLED的I2C分页刷新会占用主循环数毫秒。
 * 使用RX中断把每个字节先放进环形缓冲，主循环随后仍按原逻辑解析完整帧。
 */
void Motor_UART_EnableRxInterrupt(void)
{
    g_motor_uart_rx_head = 0U;
    g_motor_uart_rx_tail = 0U;

    /*
     * MCU和电机同时上电时，RX FIFO内可能残留不完整的上电噪声或旧反馈半帧。
     * 在启动同步查询前丢弃这些字节，确保后续解析从新的Addr(0x01)开始。
     */
    while (!DL_UART_Main_isRXFIFOEmpty(MOTOR_UART_INST)) {
        (void)DL_UART_Main_receiveData(MOTOR_UART_INST);
    }
    DL_UART_Main_setRXFIFOThreshold(
        MOTOR_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(MOTOR_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(MOTOR_UART_INST_INT_IRQN);
}

void MOTOR_UART_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(MOTOR_UART_INST) == DL_UART_IIDX_RX) {
        while (!DL_UART_Main_isRXFIFOEmpty(MOTOR_UART_INST)) {
            uint8_t received = DL_UART_Main_receiveData(MOTOR_UART_INST);
            uint8_t next = (uint8_t)((g_motor_uart_rx_head + 1U) %
                MOTOR_UART_RX_BUFFER_SIZE);

            /* 缓冲满时丢弃最旧字节，保证最新一帧仍有机会被解析。 */
            if (next == g_motor_uart_rx_tail) {
                g_motor_uart_rx_tail = (uint8_t)((g_motor_uart_rx_tail + 1U) %
                    MOTOR_UART_RX_BUFFER_SIZE);
            }
            g_motor_uart_rx_buffer[g_motor_uart_rx_head] = received;
            g_motor_uart_rx_head = next;
        }
    }
}

/**
 * @brief 通过地猛星独立UART2 TX连续发送一条Emm_V5命令。
 * @return 参数有效时返回1；空指针或长度为0时返回0。
 */
uint8_t Serial1_SendArrayTry(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((data == 0) || (length == 0U)) {
        return 0U;
    }

    for (index = 0U; index < length; index++) {
        DL_UART_transmitDataBlocking(MOTOR_UART_INST, data[index]);
    }
    return 1U;
}

/** 返回因接收缓存已满而覆盖旧数据的累计次数。 */
uint16_t UART_get_rx_overflow_count(void)
{
    return g_uart_rx_overflow_count;
}

/** 开启UART3车辆数据接收；中断只保存字节，协议解析留在主循环执行。 */
void Vehicle_UART_EnableRxInterrupt(void)
{
    g_vehicle_uart_rx_head = 0U;
    g_vehicle_uart_rx_tail = 0U;
    g_vehicle_uart_rx_overflow_count = 0U;
    g_vehicle_uart_error_count = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(VEHICLE_UART_INST)) {
        (void)DL_UART_Main_receiveData(VEHICLE_UART_INST);
    }
    DL_UART_Main_setRXFIFOThreshold(
        VEHICLE_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_clearInterruptStatus(VEHICLE_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | VEHICLE_UART_ERROR_INTERRUPTS);
    DL_UART_Main_enableInterrupt(VEHICLE_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | VEHICLE_UART_ERROR_INTERRUPTS);
    NVIC_ClearPendingIRQ(VEHICLE_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(VEHICLE_UART_INST_INT_IRQN);
}

/** 从UART3环形缓冲取出一个车辆协议字节。 */
uint8_t Vehicle_UART_ReadByte(uint8_t *value)
{
    if ((value == 0) ||
        (g_vehicle_uart_rx_head == g_vehicle_uart_rx_tail)) {
        return 0U;
    }

    *value = g_vehicle_uart_rx_buffer[g_vehicle_uart_rx_tail];
    g_vehicle_uart_rx_tail = (uint16_t)((g_vehicle_uart_rx_tail + 1U) %
        VEHICLE_UART_RX_BUFFER_SIZE);
    return 1U;
}

/** 返回因主循环处理不及时造成的车辆UART缓冲溢出次数。 */
uint16_t Vehicle_UART_GetOverflowCount(void)
{
    return g_vehicle_uart_rx_overflow_count;
}

/** 返回UART3累计硬件接收错误次数，用于判断电平、共地和波特率问题。 */
uint16_t Vehicle_UART_GetErrorCount(void)
{
    return g_vehicle_uart_error_count;
}

/** UART3车辆接收中断：一次读空FIFO，避免100Hz连续帧丢字节。 */
void VEHICLE_UART_INST_IRQHandler(void)
{
    uint32_t errors = DL_UART_Main_getRawInterruptStatus(
        VEHICLE_UART_INST, VEHICLE_UART_ERROR_INTERRUPTS);

    if (errors != 0U) {
        /* 错误状态必须主动清除，否则后续RX中断可能一直被错误原因占据。 */
        DL_UART_Main_clearInterruptStatus(VEHICLE_UART_INST, errors);
        g_vehicle_uart_error_count++;
    }

    /* 不依赖最高优先级中断原因，任何一次进入中断都先把RX FIFO读空。 */
    while (!DL_UART_Main_isRXFIFOEmpty(VEHICLE_UART_INST)) {
        const uint8_t received =
            DL_UART_Main_receiveData(VEHICLE_UART_INST);
        const uint16_t next = (uint16_t)(
            (g_vehicle_uart_rx_head + 1U) % VEHICLE_UART_RX_BUFFER_SIZE);

        if (next == g_vehicle_uart_rx_tail) {
            g_vehicle_uart_rx_tail = (uint16_t)(
                (g_vehicle_uart_rx_tail + 1U) %
                VEHICLE_UART_RX_BUFFER_SIZE);
            g_vehicle_uart_rx_overflow_count++;
        }
        g_vehicle_uart_rx_buffer[g_vehicle_uart_rx_head] = received;
        g_vehicle_uart_rx_head = next;
    }
    DL_UART_Main_clearInterruptStatus(
        VEHICLE_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
}

/** UART RX中断只负责收数，不执行发送或OLED刷新。 */
void PRINT_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(PRINT_INST) == DL_UART_IIDX_RX) {
        /* 一次中断读空硬件FIFO，避免连续数据到来时遗漏字节。 */
        while (!DL_UART_Main_isRXFIFOEmpty(PRINT_INST)) {
            const uint8_t received = DL_UART_Main_receiveData(PRINT_INST);
            const uint16_t next =
                (uint16_t)((g_uart_rx_head + 1U) % UART_RX_BUFFER_SIZE);

            if (next == g_uart_rx_tail) {
                g_uart_rx_tail =
                    (uint16_t)((g_uart_rx_tail + 1U) % UART_RX_BUFFER_SIZE);
                g_uart_rx_overflow_count++;
            }
            g_uart_rx_buffer[g_uart_rx_head] = received;
            g_uart_rx_head = next;
        }
    }
}

