#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

void board_system_clock_config(void);
void board_peripherals_init(void);
void board_motor_set_percent(float left_percent, float right_percent);
void board_motor_stop(void);
void board_motor_force_stop_from_fault(void);
void board_encoder_sample(int32_t *left_delta, int32_t *right_delta);
void board_led_set(uint8_t on);
bool board_estop_active(void);
void board_fatal_error(void);

#endif /* BOARD_H */
