#ifndef UART_TRANSPORT_H
#define UART_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    volatile uint32_t rx_overflows;
    volatile uint32_t uart_errors;
    volatile uint32_t tx_queue_overflows;
    volatile uint32_t rx_bytes;
    volatile uint32_t tx_frames;
} uart_transport_stats_t;

void uart_transport_init(void);
bool uart_transport_read_byte(uint8_t *out);
bool uart_transport_send(const uint8_t *data, size_t length);
const uart_transport_stats_t *uart_transport_get_stats(void);

#endif /* UART_TRANSPORT_H */

