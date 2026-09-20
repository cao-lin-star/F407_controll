#include "chassis_app.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "board_config.h"
#include "control.h"
#include "debug_service.h"
#include "protocol.h"
#include "ps2_remote.h"
#include "sensor_hub.h"
#include "uart_transport.h"

typedef struct {
    uint32_t cmd_vel_frames;
    uint32_t stop_frames;
    uint32_t odom_frames;
    uint32_t heartbeat_frames;
    uint32_t range_frames;
    uint32_t imu_frames;
    uint32_t debug_frames;
    uint32_t unknown_frames;
    uint16_t last_rx_seq;
} app_stats_t;

static chassis_control_t chassis;
static chassis_control_t published_chassis;
static protocol_parser_t parser;
static volatile app_stats_t app_stats;
static volatile uint8_t tx_overflow_pending;
static uint16_t tx_sequence;
static uint32_t next_control_ms;
static uint32_t next_odom_ms;
static uint32_t next_heartbeat_ms;
static uint32_t next_range_ms;
static uint32_t next_imu_ms;
static uint32_t next_debug_ms;
static uint32_t last_uart_fault_ms;
static uint32_t last_crc_fault_ms;
static uint32_t last_format_fault_ms;
static uint32_t last_tx_fault_ms;
static uint32_t last_overrun_fault_ms;

static void publish_chassis(void)
{
    taskENTER_CRITICAL();
    published_chassis = chassis;
    taskEXIT_CRITICAL();
}

static void copy_published_chassis(chassis_control_t *out)
{
    taskENTER_CRITICAL();
    *out = published_chassis;
    taskEXIT_CRITICAL();
}

static void send_payload(uint8_t msg_type,
                         const uint8_t *payload,
                         uint16_t payload_len)
{
    uint8_t frame[PROTOCOL_MAX_FRAME_SIZE];
    const size_t length = protocol_build_frame(msg_type,
                                                tx_sequence++,
                                                payload,
                                                payload_len,
                                                frame,
                                                sizeof(frame));
    if (length == 0U || !uart_transport_send(frame, length)) {
        tx_overflow_pending = 1U;
    }
}

static void send_odom(const chassis_control_t *state)
{
    uint8_t payload[28];

    protocol_write_f32_le(&payload[0], state->odom.x_m);
    protocol_write_f32_le(&payload[4], state->odom.y_m);
    protocol_write_f32_le(&payload[8], state->odom.yaw_rad);
    protocol_write_f32_le(&payload[12], state->odom.linear_mps);
    protocol_write_f32_le(&payload[16], state->odom.angular_rps);
    protocol_write_i32_le(&payload[20], state->total_left_ticks);
    protocol_write_i32_le(&payload[24], state->total_right_ticks);
    send_payload(MSG_ODOM, payload, sizeof(payload));
    ++app_stats.odom_frames;
}

static void send_heartbeat(const chassis_control_t *state, uint32_t now_ms)
{
    uint8_t payload[6];

    protocol_write_u32_le(&payload[0], now_ms);
    protocol_write_u16_le(&payload[4], state->fault_flags);
    send_payload(MSG_HEARTBEAT, payload, sizeof(payload));
    const uint8_t source = (uint8_t)state->active_source;
    send_payload(MSG_CONTROL_SOURCE, &source, 1U);
    ++app_stats.heartbeat_frames;
}

static void send_range_status(void)
{
    sensor_snapshot_t sensors;
    uint8_t payload[16];

    sensor_hub_get_snapshot(&sensors);
    protocol_write_f32_le(&payload[0], sensors.tof_left_m);
    protocol_write_f32_le(&payload[4], sensors.tof_right_m);
    protocol_write_f32_le(&payload[8], sensors.ultrasonic_m);
    protocol_write_u16_le(&payload[12], sensors.valid_flags);
    protocol_write_u16_le(&payload[14], sensors.obstacle_flags
                                      | (uint16_t)(sensors.sensor_fault_flags << 8U));
    send_payload(MSG_RANGE_STATUS, payload, sizeof(payload));
    ++app_stats.range_frames;
}

static void send_imu(void)
{
    sensor_snapshot_t sensors;
    uint8_t payload[28];

    sensor_hub_get_snapshot(&sensors);
    protocol_write_u32_le(&payload[0], sensors.imu_stamp_ms);
    protocol_write_f32_le(&payload[4], sensors.accel_x_mps2);
    protocol_write_f32_le(&payload[8], sensors.accel_y_mps2);
    protocol_write_f32_le(&payload[12], sensors.accel_z_mps2);
    protocol_write_f32_le(&payload[16], sensors.gyro_x_rps);
    protocol_write_f32_le(&payload[20], sensors.gyro_y_rps);
    protocol_write_f32_le(&payload[24], sensors.gyro_z_rps);
    send_payload(MSG_IMU_RAW, payload, sizeof(payload));
    ++app_stats.imu_frames;
}

static void send_debug_status(const chassis_control_t *state, uint32_t now_ms)
{
#if DEBUG_STATUS_ENABLE
    sensor_snapshot_t sensors;
    uint8_t payload[64];

    sensor_hub_get_snapshot(&sensors);
    protocol_write_u32_le(&payload[0], now_ms);
    protocol_write_f32_le(&payload[4], state->target_left_mm_s);
    protocol_write_f32_le(&payload[8], state->target_right_mm_s);
    protocol_write_f32_le(&payload[12], state->measured_left_mm_s);
    protocol_write_f32_le(&payload[16], state->measured_right_mm_s);
    protocol_write_f32_le(&payload[20], state->output_left_percent);
    protocol_write_f32_le(&payload[24], state->output_right_percent);
    protocol_write_i32_le(&payload[28], state->total_left_ticks);
    protocol_write_i32_le(&payload[32], state->total_right_ticks);
    protocol_write_f32_le(&payload[36], state->odom.linear_mps);
    protocol_write_f32_le(&payload[40], state->odom.angular_rps);
    protocol_write_f32_le(&payload[44], sensors.tof_left_m);
    protocol_write_f32_le(&payload[48], sensors.tof_right_m);
    protocol_write_f32_le(&payload[52], sensors.ultrasonic_m);
    protocol_write_u16_le(&payload[56], state->fault_flags);
    protocol_write_u16_le(&payload[58], sensors.valid_flags);
    protocol_write_u16_le(&payload[60], sensors.obstacle_flags);
    protocol_write_u16_le(&payload[62], sensors.sensor_fault_flags);
    send_payload(MSG_DEBUG_STATUS, payload, sizeof(payload));
    ++app_stats.debug_frames;
#else
    (void)state;
    (void)now_ms;
#endif
}

static void note_transient_fault(uint16_t fault, uint32_t now_ms)
{
    const uint32_t stamp_ms = (now_ms == 0U) ? 1U : now_ms;
    switch (fault) {
    case FAULT_UART_RX:
        last_uart_fault_ms = stamp_ms;
        break;
    case FAULT_PROTOCOL_CRC:
        last_crc_fault_ms = stamp_ms;
        break;
    case FAULT_PROTOCOL_FORMAT:
        last_format_fault_ms = stamp_ms;
        break;
    case FAULT_TX_OVERFLOW:
        last_tx_fault_ms = stamp_ms;
        break;
    case FAULT_CONTROL_OVERRUN:
        last_overrun_fault_ms = stamp_ms;
        break;
    default:
        return;
    }
    control_add_fault(&chassis, fault);
}

static void clear_transient_fault_if_quiet(uint16_t fault,
                                           uint32_t *last_seen_ms,
                                           uint32_t now_ms)
{
    if (*last_seen_ms != 0U
        && (int32_t)(now_ms - *last_seen_ms)
           >= (int32_t)TRANSIENT_FAULT_CLEAR_MS) {
        control_clear_fault(&chassis, fault);
        *last_seen_ms = 0U;
    }
}

static void update_transient_fault_recovery(uint32_t now_ms)
{
    clear_transient_fault_if_quiet(FAULT_UART_RX, &last_uart_fault_ms, now_ms);
    clear_transient_fault_if_quiet(FAULT_PROTOCOL_CRC, &last_crc_fault_ms, now_ms);
    clear_transient_fault_if_quiet(FAULT_PROTOCOL_FORMAT, &last_format_fault_ms, now_ms);
    clear_transient_fault_if_quiet(FAULT_TX_OVERFLOW, &last_tx_fault_ms, now_ms);
    clear_transient_fault_if_quiet(FAULT_CONTROL_OVERRUN, &last_overrun_fault_ms, now_ms);
}

static void process_frame(const protocol_frame_t *frame, uint32_t now_ms)
{
    app_stats.last_rx_seq = frame->seq;
    switch (frame->msg_type) {
    case MSG_CMD_VEL:
        if (frame->payload_len == 8U) {
            const float linear_mps = protocol_read_f32_le(&frame->payload[0]);
            const float angular_rps = protocol_read_f32_le(&frame->payload[4]);
            (void)control_accept_cmd_vel_from(&chassis,
                                              CONTROL_SOURCE_RK,
                                              linear_mps,
                                              angular_rps,
                                              now_ms);
            ++app_stats.cmd_vel_frames;
        } else {
            note_transient_fault(FAULT_PROTOCOL_FORMAT, now_ms);
        }
        break;

    case MSG_STOP:
        if (frame->payload_len == 0U) {
            ps2_remote_force_stop();
            control_request_fault_recovery(&chassis);
            ++app_stats.stop_frames;
        } else {
            note_transient_fault(FAULT_PROTOCOL_FORMAT, now_ms);
        }
        break;

    case MSG_HEARTBEAT:
        if (frame->payload_len != 6U) {
            note_transient_fault(FAULT_PROTOCOL_FORMAT, now_ms);
        }
        break;

    default:
        ++app_stats.unknown_frames;
        note_transient_fault(FAULT_PROTOCOL_FORMAT, now_ms);
        break;
    }
}

static void poll_uart(uint32_t now_ms)
{
    uint8_t byte;
    protocol_frame_t frame;
    uint16_t budget = 512U;

    while (budget-- > 0U && uart_transport_read_byte(&byte)) {
        if (protocol_parser_feed(&parser, byte, &frame)) {
            process_frame(&frame, now_ms);
        }
    }
}

static void update_faults_from_statistics(uint32_t now_ms)
{
    static uint32_t previous_crc_errors;
    static uint32_t previous_format_errors;
    static uint32_t previous_uart_errors;
    static uint32_t previous_rx_overflows;
    static uint32_t previous_tx_overflows;
    const uart_transport_stats_t *uart_stats = uart_transport_get_stats();
    const uint32_t format_errors = parser.stats.length_errors
                                 + parser.stats.version_errors;

    if (parser.stats.crc_errors != previous_crc_errors) {
        note_transient_fault(FAULT_PROTOCOL_CRC, now_ms);
        previous_crc_errors = parser.stats.crc_errors;
    }
    if (format_errors != previous_format_errors) {
        note_transient_fault(FAULT_PROTOCOL_FORMAT, now_ms);
        previous_format_errors = format_errors;
    }
    if (uart_stats->uart_errors != previous_uart_errors
        || uart_stats->rx_overflows != previous_rx_overflows) {
        note_transient_fault(FAULT_UART_RX, now_ms);
        previous_uart_errors = uart_stats->uart_errors;
        previous_rx_overflows = uart_stats->rx_overflows;
    }
    if (uart_stats->tx_queue_overflows != previous_tx_overflows
        || tx_overflow_pending != 0U) {
        tx_overflow_pending = 0U;
        note_transient_fault(FAULT_TX_OVERFLOW, now_ms);
        previous_tx_overflows = uart_stats->tx_queue_overflows;
    }
}

static void update_safety_interlocks(void)
{
    sensor_snapshot_t safety_sensors;
    sensor_hub_get_snapshot(&safety_sensors);
    /* Cliff: never rotate toward an unsupported wheel. Only both-wheel
     * reverse/zero is allowed until the ground flags clear. Front sonar
     * protects positive translation; pure rotation remains available. */
    const bool any_wheel_forward = chassis.debug_pwm_active
        ? (chassis.debug_pwm_left_percent > 0.0f || chassis.debug_pwm_right_percent > 0.0f)
        : (chassis.requested_left_mm_s > 0.0f || chassis.requested_right_mm_s > 0.0f);
    const bool forward_motion_requested = chassis.debug_pwm_active
        ? ((chassis.debug_pwm_left_percent
            + chassis.debug_pwm_right_percent) > 0.0f)
        : ((chassis.requested_left_mm_s
            + chassis.requested_right_mm_s) > 0.0f);

    if (board_estop_active()) {
        ps2_remote_force_stop();
        control_raise_stop_fault(&chassis, FAULT_ESTOP);
    } else {
        control_clear_fault(&chassis, FAULT_ESTOP);
    }
    const bool cliff_blocked = CLIFF_SAFETY_ENABLE && any_wheel_forward
        && (safety_sensors.obstacle_flags & (OBSTACLE_CLIFF_LEFT | OBSTACLE_CLIFF_RIGHT));
    if (cliff_blocked || (forward_motion_requested && sensor_hub_obstacle_stop_required())) {
        ps2_remote_force_stop();
        control_raise_stop_fault(&chassis, FAULT_OBSTACLE);
    } else {
        control_clear_fault(&chassis, FAULT_OBSTACLE);
    }
}

static void update_status_led(const chassis_control_t *state, uint32_t now_ms)
{
    static uint32_t previous_toggle_ms;
    static uint8_t led_on;
    uint32_t interval_ms = 500U;

    if ((state->fault_flags & (FAULT_LEFT_ENCODER | FAULT_RIGHT_ENCODER
                               | FAULT_ESTOP)) != 0U) {
        interval_ms = 100U;
    } else if (state->fault_flags != 0U) {
        interval_ms = 250U;
    }
    if ((uint32_t)(now_ms - previous_toggle_ms) >= interval_ms) {
        previous_toggle_ms = now_ms;
        led_on ^= 1U;
        board_led_set(led_on);
    }
}

void chassis_app_init(void)
{
    uint32_t now_ms;

    board_peripherals_init();
    uart_transport_init();
    sensor_hub_init();
    protocol_parser_init(&parser);
    now_ms = HAL_GetTick();
    control_init(&chassis, now_ms);
    ps2_remote_init(now_ms);
    debug_service_init(now_ms);

#if FRONT_OBSTACLE_SAFETY_ENABLE &&     (ULTRASONIC_PROTOCOL_MODE == ULTRASONIC_MODE_DISABLED)
    control_add_fault(&chassis, FAULT_SENSOR_CONFIG);
#endif
#if CLIFF_SAFETY_ENABLE && (TOF_PROTOCOL_MODE == TOF_PROTOCOL_DISABLED)
    control_add_fault(&chassis, FAULT_SENSOR_CONFIG);
#endif

    next_control_ms = now_ms + CONTROL_PERIOD_MS;
    next_odom_ms = now_ms + ODOM_PERIOD_MS;
    next_heartbeat_ms = now_ms + HEARTBEAT_PERIOD_MS;
    next_range_ms = now_ms + RANGE_STATUS_PERIOD_MS;
    next_imu_ms = now_ms + IMU_PERIOD_MS;
    next_debug_ms = now_ms + DEBUG_STATUS_PERIOD_MS;
    published_chassis = chassis;
}

void chassis_app_control_process(uint32_t now_ms)
{
    int32_t left_delta;
    int32_t right_delta;

    poll_uart(now_ms);
    {
        float linear_mps = 0.0f;
        float angular_rps = 0.0f;
        const ps2_remote_action_t action =
            ps2_remote_process(now_ms, &linear_mps, &angular_rps);

        if (action == PS2_REMOTE_ACTION_STOP) {
            if (control_accept_cmd_vel_from(&chassis,
                                            CONTROL_SOURCE_PS2,
                                            0.0f,
                                            0.0f,
                                            now_ms)) {
                control_stop(&chassis, true);
            }
        } else if (action == PS2_REMOTE_ACTION_VELOCITY) {
            (void)control_accept_cmd_vel_from(&chassis,
                                              CONTROL_SOURCE_PS2,
                                              linear_mps,
                                              angular_rps,
                                              now_ms);
        } else if (action == PS2_REMOTE_ACTION_RELEASE) {
            control_release_source(&chassis, CONTROL_SOURCE_PS2);
        }
    }
    debug_service_process(&chassis, now_ms);
    update_faults_from_statistics(now_ms);
    update_transient_fault_recovery(now_ms);
    update_safety_interlocks();

    if ((int32_t)(now_ms - next_control_ms) >= 0) {
        if ((uint32_t)(now_ms - next_control_ms) > CONTROL_PERIOD_MS) {
            note_transient_fault(FAULT_CONTROL_OVERRUN, now_ms);
            control_raise_stop_fault(&chassis, FAULT_CONTROL_OVERRUN);
            next_control_ms = now_ms + CONTROL_PERIOD_MS;
        } else {
            next_control_ms += CONTROL_PERIOD_MS;
        }
        board_encoder_sample(&left_delta, &right_delta);
        control_update(&chassis, left_delta, right_delta, now_ms);
    }
    publish_chassis();
}

void chassis_app_sensor_process(uint32_t now_ms)
{
    sensor_hub_process(now_ms);
}

void chassis_app_telemetry_process(uint32_t now_ms)
{
    chassis_control_t state;

    copy_published_chassis(&state);
    if ((int32_t)(now_ms - next_odom_ms) >= 0) {
        next_odom_ms += ODOM_PERIOD_MS;
        send_odom(&state);
    }
    if ((int32_t)(now_ms - next_range_ms) >= 0) {
        next_range_ms += RANGE_STATUS_PERIOD_MS;
        send_range_status();
    }
    if ((int32_t)(now_ms - next_imu_ms) >= 0) {
        next_imu_ms += IMU_PERIOD_MS;
        send_imu();
    }
    if ((int32_t)(now_ms - next_debug_ms) >= 0) {
        next_debug_ms += DEBUG_STATUS_PERIOD_MS;
        send_debug_status(&state, now_ms);
    }
    if ((int32_t)(now_ms - next_heartbeat_ms) >= 0) {
        next_heartbeat_ms += HEARTBEAT_PERIOD_MS;
        send_heartbeat(&state, now_ms);
    }
}

void chassis_app_housekeeping_process(uint32_t now_ms)
{
    chassis_control_t state;

    copy_published_chassis(&state);
    update_status_led(&state, now_ms);
}
