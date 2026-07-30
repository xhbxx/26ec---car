#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

void UART_send_string(UART_Regs *uart, const char *str);
void UART_send_char(UART_Regs *uart, const uint8_t chr);
uint8_t Serial1_SendArrayTry(const uint8_t *data, uint16_t length);
/** 读取电机UART0/PA31收到的一个字节；无数据返回0。 */
uint8_t Motor_UART_ReadByte(uint8_t *value);

/** 开启UART0电机反馈接收中断，避免OLED刷新期间丢失反馈帧。 */
void Motor_UART_EnableRxInterrupt(void);
uint8_t UART_read_received_byte(uint8_t *value);
uint16_t UART_get_rx_overflow_count(void);

#endif /* UART_H */
