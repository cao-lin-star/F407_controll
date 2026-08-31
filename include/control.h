#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "kinematics.h"
#include "pid.h"

enum {
    FAULT_CMD_TIMEOUT       = (1U << 0),
    FAULT_UART_RX           = (1U << 1),
    FAULT_PROTOCOL_CRC      = (1U << 2),
    FAULT_PROTOCOL_FORMAT   = (1U << 3),
    FAULT_LEFT_ENCODER      = (1U << 4),
    FAULT_RIGHT_ENCODER     = (1U << 5),
    FAULT_TX_OVERFLOW       = (1U << 6),
    FAULT_CONTROL_OVERRUN   = (1U << 7),
    FAULT_INVALID_COMMAND   = (1U << 8),
    FAULT_ESTOP             = (1U << 9),
    FAULT_OBSTACLE          = (1U << 10),
    FAULT_SENSOR_CONFIG     = (1U << 11)
};

typedef enum {
    CONTROL_SOURCE_NONE = 0,
    CONTROL_SOURCE_RK = 1,
    CONTROL_SOURCE_DEBUG = 2,
    CONTROL_SOURCE_PS2 = 3
} control_source_t;

typedef struct {
    pid_controller_t left_pid;
    pid_controller_t right_pid;
    odometry_t odom;
    float target_left_mm_s;
    float target_right_mm_s;
    float requested_left_mm_s;
    float requested_right_mm_s;
    float measured_left_mm_s;
    float measured_right_mm_s;
    float output_left_percent;
    float output_right_percent;
    float debug_pwm_left_percent;
    float debug_pwm_right_percent;
    int32_t total_left_ticks;
    int32_t total_right_ticks;
    uint32_t last_cmd_ms;
    uint32_t left_stall_since_ms;
    uint32_t right_stall_since_ms;
    uint32_t encoder_recovery_since_ms;
    uint16_t fault_flags;
    bool command_valid;
    control_source_t active_source;
    bool motion_recovery_required;
    bool recovery_neutral_seen;
    bool debug_pwm_active;
} chassis_control_t;

void control_init(chassis_control_t *control, uint32_t now_ms);
bool control_accept_cmd_vel(chassis_control_t *control,
                            float linear_mps,
                            float angular_rps,
                            uint32_t now_ms);
bool control_accept_cmd_vel_from(chassis_control_t *control,
                                 control_source_t source,
                                 float linear_mps,
                                 float angular_rps,
                                 uint32_t now_ms);
bool control_accept_debug_pwm(chassis_control_t *control,
                              float left_percent,
                              float right_percent,
                              uint32_t now_ms);
bool control_accept_debug_pwm_from(chassis_control_t *control,
                                   control_source_t source,
                                   float left_percent,
                                   float right_percent,
                                   uint32_t now_ms);
void control_release_source(chassis_control_t *control,
                            control_source_t source);
void control_stop(chassis_control_t *control, bool invalidate_command);
void control_update(chassis_control_t *control,
                    int32_t left_delta_ticks,
                    int32_t right_delta_ticks,
                    uint32_t now_ms);
void control_add_fault(chassis_control_t *control, uint16_t fault);
void control_clear_fault(chassis_control_t *control, uint16_t fault);
void control_raise_stop_fault(chassis_control_t *control, uint16_t fault);
void control_request_fault_recovery(chassis_control_t *control);

#endif /* CONTROL_H */
