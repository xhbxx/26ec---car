#include "uart_io.h"

#include <stddef.h>

/** 阻塞发送一个字节，返回前确保字节已经写入指定 UART。 */
void uart_send_byte(UART_Regs *uart, uint8_t value)
{
    DL_UART_transmitDataBlocking(uart, value);
}

/** 通过指定 UART 阻塞发送一段二进制数据。 */
void uart_send_buffer(UART_Regs *uart, const uint8_t *data, uint16_t length)
{
    if ((uart == NULL) || (data == NULL)) {
        return;
    }
    for (uint16_t i = 0U; i < length; ++i) {
        uart_send_byte(uart, data[i]);
    }
}

/** 通过指定 UART 发送以 '\0' 结尾的字符串。 */
void uart_send_string(UART_Regs *uart, const char *text)
{
    if ((uart == NULL) || (text == NULL)) {
        return;
    }
    while (*text != '\0') {
        uart_send_byte(uart, (uint8_t)*text++);
    }
}
