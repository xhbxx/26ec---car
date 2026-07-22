#ifndef UART_IO_H
#define UART_IO_H

#include "ti_msp_dl_config.h"

#include <stdint.h>

void uart_send_byte(UART_Regs *uart, uint8_t value);
void uart_send_buffer(UART_Regs *uart, const uint8_t *data, uint16_t length);
void uart_send_string(UART_Regs *uart, const char *text);

#endif /* UART_IO_H */
