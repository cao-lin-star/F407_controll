#ifndef SENSOR_HUB_H
#define SENSOR_HUB_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

enum {
    RANGE_VALID_TOF_LEFT   = (1U << 0),
    RANGE_VALID_TOF_RIGHT  = (1U << 1),
    RANGE_VALID_ULTRASONIC = (1U << 2),
    RANGE_VALID_IMU        = (1U << 3)
};

enum {
    OBSTACLE_CLIFF_LEFT  = (1U << 0),
    OBSTACLE_CLIFF_RIGHT = (1U << 1),
    OBSTACLE_ULTRASONIC  = (1U << 2),
    OBSTACLE_IR_LEFT     = (1U << 3),
    OBSTACLE_IR_RIGHT    = (1U << 4)
};

/* Compatibility aliases: bits 0/1 now describe downward cliff detectors. */
#define OBSTACLE_TOF_LEFT  OBSTACLE_CLIFF_LEFT
#define OBSTACLE_TOF_RIGHT OBSTACLE_CLIFF_RIGHT

enum {
    SENSOR_FAULT_MPU6050        = (1U << 0),
    SENSOR_FAULT_TOF_UNVERIFIED = (1U << 1),
    SENSOR_FAULT_US_UNVERIFIED  = (1U << 2),
    SENSOR_FAULT_TOF_LEFT       = (1U << 3),
    SENSOR_FAULT_TOF_RIGHT      = (1U << 4),
    SENSOR_FAULT_ULTRASONIC     = (1U << 5)
};

typedef struct {
    float tof_left_m;
    float tof_right_m;
    float ultrasonic_m;
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_rps;
    float gyro_y_rps;
    float gyro_z_rps;
    uint32_t imu_stamp_ms;
    uint32_t tof_left_rx_errors;
    uint32_t tof_right_rx_errors;
    uint32_t tof_left_frames;
    uint32_t tof_right_frames;
    uint32_t tof_left_age_ms;
    uint32_t tof_right_age_ms;
    uint16_t valid_flags;
    uint16_t obstacle_flags;
    uint16_t sensor_fault_flags;
    uint8_t tof_left_valid_zones;
    uint8_t tof_right_valid_zones;
    uint8_t tof_left_zone_count;
    uint8_t tof_right_zone_count;
    uint8_t imu_who_am_i;
} sensor_snapshot_t;

void sensor_hub_init(void);
void sensor_hub_process(uint32_t now_ms);
void sensor_hub_get_snapshot(sensor_snapshot_t *out);
bool sensor_hub_obstacle_stop_required(void);
void sensor_hub_on_uart_rx_complete(UART_HandleTypeDef *uart);
void sensor_hub_on_uart_error(UART_HandleTypeDef *uart);
void sensor_hub_on_tim_ic_capture(TIM_HandleTypeDef *timer);

#endif /* SENSOR_HUB_H */
