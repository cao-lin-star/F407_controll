#include "debug_service.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "debug_console.h"
#include "pid.h"
#include "ps2_remote.h"
#include "sensor_hub.h"

#define DEBUG_COMMAND_BUDGET 4U
#define DEBUG_PID_GAIN_MAX 10.0f

static uint32_t next_telemetry_ms;
static uint8_t stream_enabled = 1U;

static int32_t scaled_i32(float value, float scale)
{
    float scaled;

    if (!isfinite(value)) {
        return 0;
    }
    scaled = value * scale;
    if (scaled > 2147483000.0f) {
        return 2147483000L;
    }
    if (scaled < -2147483000.0f) {
        return -2147483000L;
    }
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static char *take_token(char **cursor)
{
    char *start;

    while (**cursor == ' ' || **cursor == '\t') {
        ++(*cursor);
    }
    if (**cursor == '\0') {
        return NULL;
    }
    start = *cursor;
    while (**cursor != '\0' && **cursor != ' ' && **cursor != '\t') {
        ++(*cursor);
    }
    if (**cursor != '\0') {
        **cursor = '\0';
        ++(*cursor);
    }
    return start;
}

static bool parse_decimal(const char *text, float *out)
{
    float value = 0.0f;
    float fraction = 0.1f;
    int sign = 1;
    uint8_t have_digit = 0U;

    if (text == NULL || out == NULL) {
        return false;
    }
    if (*text == '-' || *text == '+') {
        if (*text == '-') {
            sign = -1;
        }
        ++text;
    }
    while (*text >= '0' && *text <= '9') {
        have_digit = 1U;
        value = value * 10.0f + (float)(*text - '0');
        if (value > 1000000.0f) {
            return false;
        }
        ++text;
    }
    if (*text == '.') {
        ++text;
        while (*text >= '0' && *text <= '9') {
            have_digit = 1U;
            value += (float)(*text - '0') * fraction;
            fraction *= 0.1f;
            ++text;
        }
    }
    if (have_digit == 0U || *text != '\0') {
        return false;
    }
    *out = (float)sign * value;
    return isfinite(*out);
}

static void send_telemetry(const chassis_control_t *chassis, uint32_t now_ms)
{
    sensor_snapshot_t sensors;
    ps2_remote_status_t ps2;
    char line[640];
    int length;

    sensor_hub_get_snapshot(&sensors);
    ps2_remote_get_status(&ps2);
    length = snprintf(line,
                      sizeof(line),
                      "TEL,t_ms=%lu,tgt_l_mm_s=%ld,tgt_r_mm_s=%ld,"
                      "spd_l_mm_s=%ld,spd_r_mm_s=%ld,pwm_l_x100=%ld,"
                      "pwm_r_x100=%ld,enc_l=%ld,enc_r=%ld,"
                      "odom_v_mm_s=%ld,odom_w_mrad_s=%ld,tof_l_mm=%ld,"
                      "tof_r_mm=%ld,us_mm=%ld,ax_mg=%ld,ay_mg=%ld,"
                      "az_mg=%ld,gx_mrad_s=%ld,gy_mrad_s=%ld,gz_mrad_s=%ld,"
                      "open_loop=%u,source=%u,ps2_link=%u,ps2_mode=0x%02X,"
                      "ps2_manual=%u,ps2_r1=%u,ps2_stop=%u,"
                      "ps2_rx=%u,ps2_ly=%u,"
                      "fault=0x%04X,valid=0x%04X,obstacle=0x%04X,"
                      "sensor_fault=0x%04X,tof_l_rxerr=%lu,tof_r_rxerr=%lu,"
                      "tof_l_frames=%lu,tof_r_frames=%lu,"
                      "tof_l_zones=%u,tof_r_zones=%u,"
                      "tof_l_age_ms=%lu,tof_r_age_ms=%lu\r\n",
                      (unsigned long)now_ms,
                      (long)scaled_i32(chassis->target_left_mm_s, 1.0f),
                      (long)scaled_i32(chassis->target_right_mm_s, 1.0f),
                      (long)scaled_i32(chassis->measured_left_mm_s, 1.0f),
                      (long)scaled_i32(chassis->measured_right_mm_s, 1.0f),
                      (long)scaled_i32(chassis->output_left_percent, 100.0f),
                      (long)scaled_i32(chassis->output_right_percent, 100.0f),
                      (long)chassis->total_left_ticks,
                      (long)chassis->total_right_ticks,
                      (long)scaled_i32(chassis->odom.linear_mps, 1000.0f),
                      (long)scaled_i32(chassis->odom.angular_rps, 1000.0f),
                      (long)scaled_i32(sensors.tof_left_m, 1000.0f),
                      (long)scaled_i32(sensors.tof_right_m, 1000.0f),
                      (long)scaled_i32(sensors.ultrasonic_m, 1000.0f),
                      (long)scaled_i32(sensors.accel_x_mps2, 101.971621f),
                      (long)scaled_i32(sensors.accel_y_mps2, 101.971621f),
                      (long)scaled_i32(sensors.accel_z_mps2, 101.971621f),
                      (long)scaled_i32(sensors.gyro_x_rps, 1000.0f),
                      (long)scaled_i32(sensors.gyro_y_rps, 1000.0f),
                      (long)scaled_i32(sensors.gyro_z_rps, 1000.0f),
                      chassis->debug_pwm_active ? 1U : 0U,
                      (unsigned int)chassis->active_source,
                      ps2.connected,
                      ps2.mode_id,
                      ps2.manual_mode,
                      ps2.deadman_held,
                      ps2.stop_latched,
                      ps2.right_x,
                      ps2.left_y,
                      chassis->fault_flags,
                      sensors.valid_flags,
                      sensors.obstacle_flags,
                      sensors.sensor_fault_flags,
                      (unsigned long)sensors.tof_left_rx_errors,
                      (unsigned long)sensors.tof_right_rx_errors,
                      (unsigned long)sensors.tof_left_frames,
                      (unsigned long)sensors.tof_right_frames,
                      sensors.tof_left_valid_zones,
                      sensors.tof_right_valid_zones,
                      (unsigned long)sensors.tof_left_age_ms,
                      (unsigned long)sensors.tof_right_age_ms);
    if (length > 0 && (size_t)length < sizeof(line)) {
        (void)debug_console_send(line);
    }
    length = snprintf(line, sizeof(line),
        "US3,front_mm=%ld,left_mm=%ld,right_mm=%ld,status_f=%u,status_l=%u,status_r=%u,age_f=%lu,age_l=%lu,age_r=%lu,seq_f=%u,seq_l=%u,seq_r=%u\r\n",
        (long)scaled_i32(sensors.ultrasonic_three_m[0],1000.0f),
        (long)scaled_i32(sensors.ultrasonic_three_m[1],1000.0f),
        (long)scaled_i32(sensors.ultrasonic_three_m[2],1000.0f),
        sensors.ultrasonic_status[0],sensors.ultrasonic_status[1],sensors.ultrasonic_status[2],
        (unsigned long)sensors.ultrasonic_age_ms[0],(unsigned long)sensors.ultrasonic_age_ms[1],
        (unsigned long)sensors.ultrasonic_age_ms[2],sensors.ultrasonic_sequence[0],
        sensors.ultrasonic_sequence[1],sensors.ultrasonic_sequence[2]);
    if (length > 0 && (size_t)length < sizeof(line)) (void)debug_console_send(line);
}

static void send_params(const chassis_control_t *chassis)
{
    char line[256];
    const int length = snprintf(line,
                                sizeof(line),
                                "PARAM,pid_l_kp_x10000=%ld,pid_l_ki_x10000=%ld,"
                                "pid_l_kd_x10000=%ld,pid_r_kp_x10000=%ld,"
                                "pid_r_ki_x10000=%ld,pid_r_kd_x10000=%ld,"
                                "wheel_track_mm=%ld,max_linear_mm_s=%ld,"
                                "max_angular_mrad_s=%ld,cmd_timeout_ms=%u\r\n",
                                (long)scaled_i32(chassis->left_pid.kp, 10000.0f),
                                (long)scaled_i32(chassis->left_pid.ki, 10000.0f),
                                (long)scaled_i32(chassis->left_pid.kd, 10000.0f),
                                (long)scaled_i32(chassis->right_pid.kp, 10000.0f),
                                (long)scaled_i32(chassis->right_pid.ki, 10000.0f),
                                (long)scaled_i32(chassis->right_pid.kd, 10000.0f),
                                (long)scaled_i32(WHEEL_TRACK_M, 1000.0f),
                                (long)scaled_i32(MAX_LINEAR_MPS, 1000.0f),
                                (long)scaled_i32(MAX_ANGULAR_RPS, 1000.0f),
                                (unsigned int)CMD_VEL_TIMEOUT_MS);
    if (length > 0 && (size_t)length < sizeof(line)) {
        (void)debug_console_send(line);
    }
}

static void send_ps2_status(void)
{
    ps2_remote_status_t ps2;
    char line[256];
    int length;

    ps2_remote_get_status(&ps2);
    length = snprintf(line,
                      sizeof(line),
                      "PS2,link=%u,mode=0x%02X,manual=%u,r1=%u,stop=%u,"
                      "btn1=0x%02X,btn2=0x%02X,rx=%u,ry=%u,lx=%u,ly=%u,"
                      "v_mm_s=%ld,w_mrad_s=%ld,valid=%lu,invalid=%lu\r\n",
                      ps2.connected,
                      ps2.mode_id,
                      ps2.manual_mode,
                      ps2.deadman_held,
                      ps2.stop_latched,
                      ps2.btn1_raw,
                      ps2.btn2_raw,
                      ps2.right_x,
                      ps2.right_y,
                      ps2.left_x,
                      ps2.left_y,
                      (long)scaled_i32(ps2.linear_mps, 1000.0f),
                      (long)scaled_i32(ps2.angular_rps, 1000.0f),
                      (unsigned long)ps2.valid_frames,
                      (unsigned long)ps2.invalid_frames);
    if (length > 0 && (size_t)length < sizeof(line)) {
        (void)debug_console_send(line);
    }
}

static void send_help(void)
{
    (void)debug_console_send(
        "HELP,status | params | ps2 | stream on|off | stop | "
        "vel <linear_mps> <angular_rps> | "
        "wheel <left_mps> <right_mps> | "
        "pwm <left_percent> <right_percent> | "
        "pid <left|right|both> <kp> <ki> <kd>\r\n"
        "NOTE,vel/wheel/pwm is one watchdog pulse; repeat at 20Hz. "
        "pwm bypasses PID, is UART4-only, and is limited to +/-35 percent. "
        "PID changes stop the motors and are RAM-only.\r\n");
}

static void set_pid(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid_reset(pid);
}

static void process_command(chassis_control_t *chassis,
                            char *line,
                            uint32_t now_ms)
{
    char *cursor = line;
    char *command = take_token(&cursor);

    if (command == NULL) {
        return;
    }
    if (strcmp(command, "help") == 0) {
        send_help();
    } else if (strcmp(command, "status") == 0) {
        send_telemetry(chassis, now_ms);
    } else if (strcmp(command, "params") == 0) {
        send_params(chassis);
    } else if (strcmp(command, "ps2") == 0) {
        send_ps2_status();
    } else if (strcmp(command, "stop") == 0) {
        ps2_remote_force_stop();
        control_request_fault_recovery(chassis);
        (void)debug_console_send("OK,stopped\r\n");
    } else if (strcmp(command, "stream") == 0) {
        char *mode = take_token(&cursor);
        if (mode != NULL && strcmp(mode, "on") == 0) {
            stream_enabled = 1U;
            next_telemetry_ms = now_ms;
            (void)debug_console_send("OK,stream=on\r\n");
        } else if (mode != NULL && strcmp(mode, "off") == 0) {
            stream_enabled = 0U;
            (void)debug_console_send("OK,stream=off\r\n");
        } else {
            (void)debug_console_send("ERR,usage: stream on|off\r\n");
        }
    } else if (strcmp(command, "vel") == 0) {
        float linear_mps;
        float angular_rps;
        char *linear = take_token(&cursor);
        char *angular = take_token(&cursor);
        if (!parse_decimal(linear, &linear_mps)
            || !parse_decimal(angular, &angular_rps)
            || take_token(&cursor) != NULL) {
            (void)debug_console_send("ERR,usage: vel <linear_mps> <angular_rps>\r\n");
        } else if (control_accept_cmd_vel_from(chassis,
                                               CONTROL_SOURCE_DEBUG,
                                               linear_mps,
                                               angular_rps,
                                               now_ms)) {
            (void)debug_console_send("OK,vel accepted; watchdog=500ms\r\n");
        } else {
            (void)debug_console_send(
                "ERR,vel rejected; higher-priority source or safety recovery active\r\n");
        }
    } else if (strcmp(command, "wheel") == 0) {
        float left_mps;
        float right_mps;
        char *left = take_token(&cursor);
        char *right = take_token(&cursor);
        if (!parse_decimal(left, &left_mps)
            || !parse_decimal(right, &right_mps)
            || take_token(&cursor) != NULL) {
            (void)debug_console_send("ERR,usage: wheel <left_mps> <right_mps>\r\n");
        } else if (fabsf(left_mps) > MAX_WHEEL_MPS
                   || fabsf(right_mps) > MAX_WHEEL_MPS) {
            (void)debug_console_send("ERR,wheel speed exceeds configured limit\r\n");
        } else if (control_accept_cmd_vel_from(
                       chassis,
                       CONTROL_SOURCE_DEBUG,
                       (left_mps + right_mps) * 0.5f,
                       (right_mps - left_mps) / WHEEL_TRACK_M,
                       now_ms)) {
            (void)debug_console_send("OK,wheel accepted; watchdog=500ms\r\n");
        } else {
            (void)debug_console_send(
                "ERR,wheel rejected; higher-priority source or safety recovery active\r\n");
        }
    } else if (strcmp(command, "pwm") == 0) {
        float left_percent;
        float right_percent;
        char *left = take_token(&cursor);
        char *right = take_token(&cursor);
        if (!parse_decimal(left, &left_percent)
            || !parse_decimal(right, &right_percent)
            || take_token(&cursor) != NULL) {
            (void)debug_console_send("ERR,usage: pwm <left_percent> <right_percent>\r\n");
        } else if (fabsf(left_percent) > MOTOR_OUTPUT_LIMIT_PERCENT
                   || fabsf(right_percent) > MOTOR_OUTPUT_LIMIT_PERCENT) {
            (void)debug_console_send("ERR,pwm exceeds +/-35 percent limit\r\n");
        } else if (control_accept_debug_pwm_from(chassis,
                                                 CONTROL_SOURCE_DEBUG,
                                                 left_percent,
                                                 right_percent,
                                                 now_ms)) {
            (void)debug_console_send(
                "OK,pwm accepted; PID bypassed; watchdog=500ms\r\n");
        } else {
            (void)debug_console_send(
                "ERR,pwm rejected; higher-priority source or safety recovery active\r\n");
        }
    } else if (strcmp(command, "pid") == 0) {
        char *side = take_token(&cursor);
        char *kp_text = take_token(&cursor);
        char *ki_text = take_token(&cursor);
        char *kd_text = take_token(&cursor);
        float kp;
        float ki;
        float kd;
        const bool side_valid = side != NULL
                             && (strcmp(side, "left") == 0
                                 || strcmp(side, "right") == 0
                                 || strcmp(side, "both") == 0);
        if (!side_valid
            || !parse_decimal(kp_text, &kp)
            || !parse_decimal(ki_text, &ki)
            || !parse_decimal(kd_text, &kd)
            || take_token(&cursor) != NULL
            || kp < 0.0f || ki < 0.0f || kd < 0.0f
            || kp > DEBUG_PID_GAIN_MAX
            || ki > DEBUG_PID_GAIN_MAX
            || kd > DEBUG_PID_GAIN_MAX) {
            (void)debug_console_send(
                "ERR,usage: pid <left|right|both> <kp> <ki> <kd>; gains 0..10\r\n");
        } else {
            ps2_remote_force_stop();
            control_request_fault_recovery(chassis);
            if (strcmp(side, "left") == 0 || strcmp(side, "both") == 0) {
                set_pid(&chassis->left_pid, kp, ki, kd);
            }
            if (strcmp(side, "right") == 0 || strcmp(side, "both") == 0) {
                set_pid(&chassis->right_pid, kp, ki, kd);
            }
            (void)debug_console_send("OK,pid updated in RAM; motors stopped\r\n");
            send_params(chassis);
        }
    } else {
        (void)debug_console_send("ERR,unknown command; type help\r\n");
    }
}

void debug_service_init(uint32_t now_ms)
{
    debug_console_init();
    stream_enabled = 1U;
    next_telemetry_ms = now_ms + DEBUG_ASCII_PERIOD_MS;
    (void)debug_console_send(
        "READY,Footbath Chassis ASCII debug UART4 115200 8N1; type help\r\n");
}

void debug_service_process(chassis_control_t *chassis, uint32_t now_ms)
{
    char line[128];
    uint8_t budget = DEBUG_COMMAND_BUDGET;

    while (budget-- > 0U && debug_console_read_line(line, sizeof(line))) {
        process_command(chassis, line, now_ms);
    }
    if (stream_enabled != 0U && (int32_t)(now_ms - next_telemetry_ms) >= 0) {
        next_telemetry_ms += DEBUG_ASCII_PERIOD_MS;
        send_telemetry(chassis, now_ms);
    }
}
