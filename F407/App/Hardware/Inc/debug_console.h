#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

void debug_console_init(void);
bool debug_console_read_line(char *out, size_t capacity);
bool debug_console_send(const char *text);
void debug_console_on_uart_rx_complete(UART_HandleTypeDef *uart);
void debug_console_on_uart_tx_complete(UART_HandleTypeDef *uart);
void debug_console_on_uart_error(UART_HandleTypeDef *uart);

#endif /* DEBUG_CONSOLE_H */
