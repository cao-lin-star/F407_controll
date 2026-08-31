#include "debug_console.h"

#include <string.h>

#include "usart.h"

#define DEBUG_RX_RING_SIZE 256U
#define DEBUG_RX_RING_MASK (DEBUG_RX_RING_SIZE - 1U)
#define DEBUG_TX_QUEUE_SLOTS 4U
#define DEBUG_TX_BUFFER_SIZE 640U

typedef char debug_rx_ring_size_must_be_power_of_two[
    ((DEBUG_RX_RING_SIZE & (DEBUG_RX_RING_SIZE - 1U)) == 0U) ? 1 : -1];

static uint8_t rx_irq_byte;
static uint8_t rx_ring[DEBUG_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static char line_buffer[128];
static size_t line_length;
static uint8_t line_discarding;

static uint8_t tx_buffers[DEBUG_TX_QUEUE_SLOTS][DEBUG_TX_BUFFER_SIZE];
static uint16_t tx_lengths[DEBUG_TX_QUEUE_SLOTS];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_active;

static uint32_t enter_critical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void leave_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void start_next_tx(void)
{
    if (tx_active != 0U || tx_tail == tx_head) {
        return;
    }
    if (HAL_UART_Transmit_IT(&huart4,
                             tx_buffers[tx_tail],
                             tx_lengths[tx_tail]) == HAL_OK) {
        tx_active = 1U;
    }
}

void debug_console_init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    line_length = 0U;
    line_discarding = 0U;
    tx_head = 0U;
    tx_tail = 0U;
    tx_active = 0U;
    if (HAL_UART_Receive_IT(&huart4, &rx_irq_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

bool debug_console_read_line(char *out, size_t capacity)
{
    while (rx_tail != rx_head) {
        const uint8_t byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & DEBUG_RX_RING_MASK);

        if (byte == '\r' || byte == '\n') {
            if (line_discarding != 0U) {
                line_discarding = 0U;
                line_length = 0U;
                continue;
            }
            if (line_length == 0U) {
                continue;
            }
            if (capacity == 0U || line_length >= capacity) {
                line_length = 0U;
                return false;
            }
            memcpy(out, line_buffer, line_length);
            out[line_length] = '\0';
            line_length = 0U;
            return true;
        }

        if (line_discarding != 0U) {
            continue;
        }
        if (byte == 0x08U || byte == 0x7FU) {
            if (line_length > 0U) {
                --line_length;
            }
            continue;
        }
        if (byte >= 0x20U && byte <= 0x7EU) {
            if (line_length < sizeof(line_buffer) - 1U) {
                line_buffer[line_length++] = (char)byte;
            } else {
                line_discarding = 1U;
                line_length = 0U;
            }
        }
    }
    return false;
}

bool debug_console_send(const char *text)
{
    size_t length;
    uint8_t next_head;
    uint32_t primask;

    if (text == NULL) {
        return false;
    }
    length = strlen(text);
    if (length == 0U || length >= DEBUG_TX_BUFFER_SIZE) {
        return false;
    }

    primask = enter_critical();
    next_head = (uint8_t)((tx_head + 1U) % DEBUG_TX_QUEUE_SLOTS);
    if (next_head == tx_tail) {
        leave_critical(primask);
        return false;
    }
    memcpy(tx_buffers[tx_head], text, length);
    tx_lengths[tx_head] = (uint16_t)length;
    tx_head = next_head;
    start_next_tx();
    leave_critical(primask);
    return true;
}

void debug_console_on_uart_rx_complete(UART_HandleTypeDef *uart)
{
    uint16_t next_head;

    if (uart != &huart4) {
        return;
    }
    next_head = (uint16_t)((rx_head + 1U) & DEBUG_RX_RING_MASK);
    if (next_head != rx_tail) {
        rx_ring[rx_head] = rx_irq_byte;
        rx_head = next_head;
    }
    (void)HAL_UART_Receive_IT(&huart4, &rx_irq_byte, 1U);
}

void debug_console_on_uart_tx_complete(UART_HandleTypeDef *uart)
{
    if (uart != &huart4) {
        return;
    }
    tx_tail = (uint8_t)((tx_tail + 1U) % DEBUG_TX_QUEUE_SLOTS);
    tx_active = 0U;
    start_next_tx();
}

void debug_console_on_uart_error(UART_HandleTypeDef *uart)
{
    if (uart != &huart4) {
        return;
    }
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    (void)HAL_UART_Receive_IT(&huart4, &rx_irq_byte, 1U);
}
