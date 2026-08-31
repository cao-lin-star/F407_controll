#include "uart_transport.h"

#include <string.h>

#include "board.h"
#include "debug_console.h"
#include "protocol.h"
#include "sensor_hub.h"
#include "usart.h"

#define RX_RING_SIZE 256U
#define RX_RING_MASK (RX_RING_SIZE - 1U)
#define TX_QUEUE_SLOTS 8U

typedef char rx_ring_size_must_be_power_of_two[
    ((RX_RING_SIZE & (RX_RING_SIZE - 1U)) == 0U) ? 1 : -1];

static uint8_t rx_irq_byte;
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t tx_buffers[TX_QUEUE_SLOTS][PROTOCOL_MAX_FRAME_SIZE];
static uint16_t tx_lengths[TX_QUEUE_SLOTS];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_active;
static uart_transport_stats_t transport_stats;

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
    if (HAL_UART_Transmit_IT(&huart1,
                             tx_buffers[tx_tail],
                             tx_lengths[tx_tail]) == HAL_OK) {
        tx_active = 1U;
    } else {
        ++transport_stats.uart_errors;
        tx_tail = (uint8_t)((tx_tail + 1U) % TX_QUEUE_SLOTS);
    }
}

void uart_transport_init(void)
{
    memset(&transport_stats, 0, sizeof(transport_stats));
    rx_head = 0U;
    rx_tail = 0U;
    tx_head = 0U;
    tx_tail = 0U;
    tx_active = 0U;
    if (HAL_UART_Receive_IT(&huart1, &rx_irq_byte, 1U) != HAL_OK) {
        board_fatal_error();
    }
}

void uart_transport_irq_handler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

bool uart_transport_read_byte(uint8_t *out)
{
    const uint16_t tail = rx_tail;
    if (tail == rx_head) {
        return false;
    }
    *out = rx_ring[tail];
    rx_tail = (uint16_t)((tail + 1U) & RX_RING_MASK);
    return true;
}

bool uart_transport_send(const uint8_t *data, size_t length)
{
    uint8_t next_head;
    uint32_t primask;

    if (data == NULL || length == 0U || length > PROTOCOL_MAX_FRAME_SIZE) {
        return false;
    }
    primask = enter_critical();
    next_head = (uint8_t)((tx_head + 1U) % TX_QUEUE_SLOTS);
    if (next_head == tx_tail) {
        ++transport_stats.tx_queue_overflows;
        leave_critical(primask);
        return false;
    }
    memcpy(tx_buffers[tx_head], data, length);
    tx_lengths[tx_head] = (uint16_t)length;
    tx_head = next_head;
    start_next_tx();
    leave_critical(primask);
    return true;
}

const uart_transport_stats_t *uart_transport_get_stats(void)
{
    return &transport_stats;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) {
        const uint16_t next_head = (uint16_t)((rx_head + 1U) & RX_RING_MASK);
        ++transport_stats.rx_bytes;
        if (next_head == rx_tail) {
            ++transport_stats.rx_overflows;
        } else {
            rx_ring[rx_head] = rx_irq_byte;
            rx_head = next_head;
        }
        if (HAL_UART_Receive_IT(&huart1, &rx_irq_byte, 1U) != HAL_OK) {
            ++transport_stats.uart_errors;
        }
    } else if (uart == &huart4) {
        debug_console_on_uart_rx_complete(uart);
    } else {
        sensor_hub_on_uart_rx_complete(uart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) {
        ++transport_stats.tx_frames;
        tx_tail = (uint8_t)((tx_tail + 1U) % TX_QUEUE_SLOTS);
        tx_active = 0U;
        start_next_tx();
    } else if (uart == &huart4) {
        debug_console_on_uart_tx_complete(uart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) {
        ++transport_stats.uart_errors;
        (void)HAL_UART_Receive_IT(&huart1, &rx_irq_byte, 1U);
    } else if (uart == &huart4) {
        debug_console_on_uart_error(uart);
    } else {
        sensor_hub_on_uart_error(uart);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *timer)
{
    sensor_hub_on_tim_ic_capture(timer);
}


