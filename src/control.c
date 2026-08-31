#include "control.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "board_config.h"

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}
static bool source_can_claim(const chassis_control_t *control,
                             control_source_t source,
                             uint32_t now_ms)
{
    if (source == CONTROL_SOURCE_NONE) {
        return false;
    }
    if (!control->command_valid
        || (uint32_t)(now_ms - control->last_cmd_ms) > CMD_VEL_TIMEOUT_MS) {
        return true;
    }
    return source >= control->active_source;
}

static float slew_towards(float current, float requested, float dt_s)
{
    float rate_mps2;
    float maximum_step;
    float delta;
    const bool reversing = current * requested < 0.0f;
    const bool increasing_magnitude = fabsf(requested) > fabsf(current);

    if (reversing) {
        requested = 0.0f;
    }
    rate_mps2 = increasing_magnitude && !reversing
              ? WHEEL_ACCEL_LIMIT_MPS2
              : WHEEL_DECEL_LIMIT_MPS2;
    maximum_step = rate_mps2 * 1000.0f * dt_s;
    delta = requested - current;
    if (delta > maximum_step) {
        return current + maximum_step;
    }
    if (delta < -maximum_step) {
        return current - maximum_step;
    }
    return requested;
}

static bool velocity_is_neutral(float linear_mps, float angular_rps)
{
    return fabsf(linear_mps) < 0.0001f && fabsf(angular_rps) < 0.0001f;
}

static bool pwm_is_neutral(float left_percent, float right_percent)
{
    return fabsf(left_percent) < 0.0001f && fabsf(right_percent) < 0.0001f;
}


static void stop_outputs(chassis_control_t *control)
{
    control->output_left_percent = 0.0f;
    control->output_right_percent = 0.0f;
    control->debug_pwm_left_percent = 0.0f;
    control->debug_pwm_right_percent = 0.0f;
    control->debug_pwm_active = false;
    pid_reset(&control->left_pid);
    pid_reset(&control->right_pid);
    board_motor_stop();
}

static float apply_static_friction_feedforward(float correction_percent,
                                               float target_mm_s,
                                               float measured_mm_s,
                                               float breakaway_percent,
                                               float running_percent)
{
    float output_percent;
    float feedforward_percent;

    if (target_mm_s == 0.0f) {
        return 0.0f;
    }
    feedforward_percent = target_mm_s * measured_mm_s > 0.0f
                        ? running_percent
                        : breakaway_percent;
    if (target_mm_s > 0.0f) {
        output_percent = correction_percent + feedforward_percent;
        if (output_percent < 0.0f) {
            output_percent = 0.0f;
        }
    } else {
        output_percent = correction_percent - feedforward_percent;
        if (output_percent > 0.0f) {
            output_percent = 0.0f;
        }
    }
    return clampf(output_percent,
                  -MOTOR_OUTPUT_LIMIT_PERCENT,
                  MOTOR_OUTPUT_LIMIT_PERCENT);
}

void control_init(chassis_control_t *control, uint32_t now_ms)
{
    memset(control, 0, sizeof(*control));
    pid_init(&control->left_pid,
             LEFT_PID_KP, LEFT_PID_KI, LEFT_PID_KD,
             PID_INTEGRAL_LIMIT_MM_S,
             -MOTOR_OUTPUT_LIMIT_PERCENT,
             MOTOR_OUTPUT_LIMIT_PERCENT);
    pid_init(&control->right_pid,
             RIGHT_PID_KP, RIGHT_PID_KI, RIGHT_PID_KD,
             PID_INTEGRAL_LIMIT_MM_S,
             -MOTOR_OUTPUT_LIMIT_PERCENT,
             MOTOR_OUTPUT_LIMIT_PERCENT);
    control->last_cmd_ms = now_ms;
    board_motor_stop();
}

bool control_accept_cmd_vel(chassis_control_t *control,
                            float linear_mps,
                            float angular_rps,
                            uint32_t now_ms)
{
    return control_accept_cmd_vel_from(control,
                                       CONTROL_SOURCE_RK,
                                       linear_mps,
                                       angular_rps,
                                       now_ms);
}

bool control_accept_cmd_vel_from(chassis_control_t *control,
                                 control_source_t source,
                                 float linear_mps,
                                 float angular_rps,
                                 uint32_t now_ms)
{
    float left_mps;
    float right_mps;
    float largest;
    if (!source_can_claim(control, source, now_ms)) {
        return false;
    }

    if (!isfinite(linear_mps) || !isfinite(angular_rps)) {
        control_raise_stop_fault(control, FAULT_INVALID_COMMAND);
        return false;
    }
    if (control->motion_recovery_required) {
        if (!velocity_is_neutral(linear_mps, angular_rps)) {
            control->recovery_neutral_seen = false;
            control->encoder_recovery_since_ms = 0U;
            control_add_fault(control, FAULT_INVALID_COMMAND);
            control_stop(control, true);
            return false;
        }
        if (!control->recovery_neutral_seen) {
            control->encoder_recovery_since_ms = 0U;
        }
        control->recovery_neutral_seen = true;
        control_clear_fault(control, FAULT_INVALID_COMMAND);
    } else {
        control_clear_fault(control, FAULT_INVALID_COMMAND);
    }

    linear_mps = clampf(linear_mps, -MAX_LINEAR_MPS, MAX_LINEAR_MPS);
    angular_rps = clampf(angular_rps, -MAX_ANGULAR_RPS, MAX_ANGULAR_RPS);
    kinematics_inverse(linear_mps, angular_rps, WHEEL_TRACK_M,

                       &left_mps, &right_mps);

    /* 保持左右轮比例，整体缩放到允许的最大轮速。 */
    largest = fmaxf(fabsf(left_mps), fabsf(right_mps));
    if (largest > MAX_WHEEL_MPS) {
        const float scale = MAX_WHEEL_MPS / largest;
        left_mps *= scale;
        right_mps *= scale;
    }


    control->requested_left_mm_s = left_mps * 1000.0f;
    control->requested_right_mm_s = right_mps * 1000.0f;
    control->debug_pwm_left_percent = 0.0f;
    control->debug_pwm_right_percent = 0.0f;
    control->debug_pwm_active = false;
    control->last_cmd_ms = now_ms;
    control->active_source = source;
    control->command_valid = true;
    control->fault_flags &= (uint16_t)~FAULT_CMD_TIMEOUT;
    return true;
}

bool control_accept_debug_pwm(chassis_control_t *control,
                              float left_percent,
                              float right_percent,
                              uint32_t now_ms)
{
    return control_accept_debug_pwm_from(control,
                                         CONTROL_SOURCE_DEBUG,
                                         left_percent,
                                         right_percent,
                                         now_ms);

}


bool control_accept_debug_pwm_from(chassis_control_t *control,
                                   control_source_t source,
                                   float left_percent,
                                   float right_percent,
                                   uint32_t now_ms)
{
    if (!source_can_claim(control, source, now_ms)) {
        return false;
    }
    if (!isfinite(left_percent) || !isfinite(right_percent)
        || fabsf(left_percent) > MOTOR_OUTPUT_LIMIT_PERCENT
        || fabsf(right_percent) > MOTOR_OUTPUT_LIMIT_PERCENT) {
        control_raise_stop_fault(control, FAULT_INVALID_COMMAND);

        return false;
    }

    if (control->motion_recovery_required) {
        if (!pwm_is_neutral(left_percent, right_percent)) {

            control->recovery_neutral_seen = false;
            control->encoder_recovery_since_ms = 0U;
            control_add_fault(control, FAULT_INVALID_COMMAND);
            control_stop(control, true);
            return false;
        }
        if (!control->recovery_neutral_seen) {
            control->encoder_recovery_since_ms = 0U;
        }
        control->recovery_neutral_seen = true;
        control_clear_fault(control, FAULT_INVALID_COMMAND);
    } else {
        control_clear_fault(control, FAULT_INVALID_COMMAND);
    }

    control->target_left_mm_s = 0.0f;
    control->target_right_mm_s = 0.0f;
    control->requested_left_mm_s = 0.0f;
    control->requested_right_mm_s = 0.0f;
    control->debug_pwm_left_percent = left_percent;
    control->debug_pwm_right_percent = right_percent;
    control->debug_pwm_active = true;
    control->last_cmd_ms = now_ms;
    control->active_source = source;
    control->command_valid = true;
    control->fault_flags &= (uint16_t)~FAULT_CMD_TIMEOUT;
    pid_reset(&control->left_pid);
    pid_reset(&control->right_pid);
    return true;
}

void control_stop(chassis_control_t *control, bool invalidate_command)
{
    control->target_left_mm_s = 0.0f;
    control->target_right_mm_s = 0.0f;
    control->requested_left_mm_s = 0.0f;
    control->requested_right_mm_s = 0.0f;
    control->left_stall_since_ms = 0U;
    control->right_stall_since_ms = 0U;
    if (invalidate_command) {
        control->command_valid = false;
        control->active_source = CONTROL_SOURCE_NONE;
    }
    stop_outputs(control);
}
void control_release_source(chassis_control_t *control,
                            control_source_t source)
{
    if (control->active_source == source) {
        control_stop(control, true);
    }
}

void control_add_fault(chassis_control_t *control, uint16_t fault)
{
    control->fault_flags |= fault;
}

void control_clear_fault(chassis_control_t *control, uint16_t fault)
{
    control->fault_flags &= (uint16_t)~fault;
}

void control_raise_stop_fault(chassis_control_t *control, uint16_t fault)
{
    control_add_fault(control, fault);
    control->motion_recovery_required = true;
    control->recovery_neutral_seen = false;
    control->encoder_recovery_since_ms = 0U;
    control_stop(control, true);
}

void control_request_fault_recovery(chassis_control_t *control)
{
    if (control->motion_recovery_required) {
        if (!control->recovery_neutral_seen) {
            control->encoder_recovery_since_ms = 0U;
        }
        control->recovery_neutral_seen = true;
        control_clear_fault(control, FAULT_INVALID_COMMAND);
    }
    control_stop(control, true);
}

static bool encoder_delta_is_stable(int32_t delta_ticks)
{
    return delta_ticks <= ENCODER_RECOVERY_MAX_DELTA_PER_PERIOD
        && delta_ticks >= -ENCODER_RECOVERY_MAX_DELTA_PER_PERIOD;
}

static bool update_stall_timer(uint32_t *since_ms,
                               float target_mm_s,
                               float output_percent,
                               int32_t delta_ticks,
                               uint32_t now_ms)
{
    const bool should_be_moving = fabsf(target_mm_s) >= ENCODER_STALL_COMMAND_MM_S
                               && fabsf(output_percent) >= ENCODER_STALL_PWM_PERCENT;
    if (should_be_moving && delta_ticks == 0) {
        if (*since_ms == 0U) {
            *since_ms = (now_ms == 0U) ? 1U : now_ms;
        } else if ((uint32_t)(now_ms - *since_ms) >= ENCODER_STALL_TIMEOUT_MS) {
            return true;
        }
    } else {
        *since_ms = 0U;
    }
    return false;
}

void control_update(chassis_control_t *control,
                    int32_t left_delta_ticks,
                    int32_t right_delta_ticks,
                    uint32_t now_ms)
{
    const float dt_s = (float)CONTROL_PERIOD_MS * 0.001f;
    float left_distance_m;
    float right_distance_m;

    left_delta_ticks *= LEFT_ENCODER_SIGN;
    right_delta_ticks *= RIGHT_ENCODER_SIGN;

    /* 单周期跳变超过物理合理范围：立即停机并进入受控恢复等待。 */
    if (left_delta_ticks > ENCODER_MAX_DELTA_PER_PERIOD
        || left_delta_ticks < -ENCODER_MAX_DELTA_PER_PERIOD) {
        control_raise_stop_fault(control, FAULT_LEFT_ENCODER);
    }
    if (right_delta_ticks > ENCODER_MAX_DELTA_PER_PERIOD
        || right_delta_ticks < -ENCODER_MAX_DELTA_PER_PERIOD) {
        control_raise_stop_fault(control, FAULT_RIGHT_ENCODER);
    }
    if (control->motion_recovery_required) {
        const uint16_t encoder_faults =
            (uint16_t)(control->fault_flags
                       & (FAULT_LEFT_ENCODER | FAULT_RIGHT_ENCODER));
        const bool encoders_stable =
            encoder_delta_is_stable(left_delta_ticks)
            && encoder_delta_is_stable(right_delta_ticks);

        if (encoder_faults != 0U) {
            if (!control->recovery_neutral_seen || !encoders_stable) {
                control->encoder_recovery_since_ms = 0U;
            } else if (control->encoder_recovery_since_ms == 0U) {
                control->encoder_recovery_since_ms =
                    (now_ms == 0U) ? 1U : now_ms;
            } else if ((uint32_t)(now_ms
                                   - control->encoder_recovery_since_ms)
                       >= ENCODER_RECOVERY_STABLE_MS) {
                control_clear_fault(control,
                                    FAULT_LEFT_ENCODER
                                    | FAULT_RIGHT_ENCODER);
                control->encoder_recovery_since_ms = 0U;
            }
        }


        if (control->recovery_neutral_seen
            && (control->fault_flags
                & (FAULT_LEFT_ENCODER | FAULT_RIGHT_ENCODER
                   | FAULT_CONTROL_OVERRUN | FAULT_INVALID_COMMAND
                   | FAULT_ESTOP | FAULT_OBSTACLE)) == 0U) {
            control->motion_recovery_required = false;
            control->recovery_neutral_seen = false;
        }

        control_stop(control, true);
        return;
    }

    control->total_left_ticks += left_delta_ticks;
    control->total_right_ticks += right_delta_ticks;
    left_distance_m = (float)left_delta_ticks * LEFT_DISTANCE_PER_TICK_M;
    right_distance_m = (float)right_delta_ticks * RIGHT_DISTANCE_PER_TICK_M;
    control->measured_left_mm_s = left_distance_m * 1000.0f / dt_s;
    control->measured_right_mm_s = right_distance_m * 1000.0f / dt_s;
    kinematics_integrate(&control->odom,
                         left_distance_m,
                         right_distance_m,
                         dt_s,
                         WHEEL_TRACK_M);

    if (!control->command_valid
        || (uint32_t)(now_ms - control->last_cmd_ms) > CMD_VEL_TIMEOUT_MS) {
        control->fault_flags |= FAULT_CMD_TIMEOUT;
        control_stop(control, true);
        return;
    }

    /* UART4-only commissioning mode: bypass both PID controllers while
     * retaining encoder sampling, odometry, safety interlocks and watchdog. */
    if (control->debug_pwm_active) {
        control->output_left_percent = clampf(control->debug_pwm_left_percent,
                                              -MOTOR_OUTPUT_LIMIT_PERCENT,
                                              MOTOR_OUTPUT_LIMIT_PERCENT);
        control->output_right_percent = clampf(control->debug_pwm_right_percent,
                                               -MOTOR_OUTPUT_LIMIT_PERCENT,
                                               MOTOR_OUTPUT_LIMIT_PERCENT);
        board_motor_set_percent(control->output_left_percent,
                                control->output_right_percent);
        return;
    }
    control->target_left_mm_s = slew_towards(control->target_left_mm_s,
                                             control->requested_left_mm_s,
                                             dt_s);
    control->target_right_mm_s = slew_towards(control->target_right_mm_s,
                                              control->requested_right_mm_s,
                                              dt_s);

    /* 零速命令绕过 PID，避免静止时积分或电机啸叫。 */
    if (fabsf(control->target_left_mm_s) < 0.01f
        && fabsf(control->target_right_mm_s) < 0.01f) {
        stop_outputs(control);
        return;
    }

    pid_set_target(&control->left_pid, control->target_left_mm_s);
    pid_set_target(&control->right_pid, control->target_right_mm_s);
    control->output_left_percent = apply_static_friction_feedforward(
        pid_update(&control->left_pid, control->measured_left_mm_s),
        control->target_left_mm_s,
        control->measured_left_mm_s,
        LEFT_BREAKAWAY_FF_PERCENT,
        LEFT_RUNNING_FF_PERCENT);
    control->output_right_percent = apply_static_friction_feedforward(
        pid_update(&control->right_pid, control->measured_right_mm_s),
        control->target_right_mm_s,
        control->measured_right_mm_s,
        RIGHT_BREAKAWAY_FF_PERCENT,
        RIGHT_RUNNING_FF_PERCENT);

    if (update_stall_timer(&control->left_stall_since_ms,
                           control->target_left_mm_s,
                           control->output_left_percent,
                           left_delta_ticks,
                           now_ms)) {
        control_raise_stop_fault(control, FAULT_LEFT_ENCODER);
    }
    if (update_stall_timer(&control->right_stall_since_ms,
                           control->target_right_mm_s,
                           control->output_right_percent,
                           right_delta_ticks,
                           now_ms)) {
        control_raise_stop_fault(control, FAULT_RIGHT_ENCODER);
    }
    if (control->motion_recovery_required) {
        control_stop(control, true);
        return;
    }

    board_motor_set_percent(control->output_left_percent,
                            control->output_right_percent);
}
