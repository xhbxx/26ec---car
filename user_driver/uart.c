#include "uart.h"

#define UART_RX_BUFFER_SIZE            (512U)

static volatile uint8_t g_uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t g_uart_rx_head = 0U;
static volatile uint16_t g_uart_rx_tail = 0U;
static volatile uint16_t g_uart_rx_overflow_count = 0U;

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

/** 从中断接收缓冲区读取一个字节；无数据返回0，成功返回1。 */
uint8_t UART_read_received_byte(uint8_t *value)
{
    if (value == 0) {
        return 0U;
    }

    /*
     * 当前UART-OLED测试程序直接轮询硬件FIFO。
     * 这样不依赖NVIC和RX中断阈值，适合先确认PA31是否真正收到数据。
     */
    return DL_UART_Main_receiveDataCheck(PRINT_INST, value) ? 1U : 0U;
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

