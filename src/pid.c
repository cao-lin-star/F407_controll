#include "pid.h"

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

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_min,
              float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid_reset(pid);
}

void pid_reset(pid_controller_t *pid)
{
    pid->target = 0.0f;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

void pid_set_target(pid_controller_t *pid, float target)
{
    pid->target = target;
}

float pid_update(pid_controller_t *pid, float measurement)
{
    const float error = pid->target - measurement;
    const float derivative = error - pid->previous_error;
    const float candidate_integral = clampf(pid->integral + error,
                                             -pid->integral_limit,
                                             pid->integral_limit);
    const float candidate_output = pid->kp * error
                                 + pid->ki * candidate_integral
                                 + pid->kd * derivative;

    /*
     * 条件积分抗饱和：输出已在上限且误差还想继续推高（或下限/推低）时，
     * 暂停积分；一旦误差方向有助于脱离饱和，立即允许积分回退。
     */
    if (!((candidate_output > pid->output_max && error > 0.0f)
          || (candidate_output < pid->output_min && error < 0.0f))) {
        pid->integral = candidate_integral;
    }

    pid->previous_error = error;
    return clampf(pid->kp * error
                  + pid->ki * pid->integral
                  + pid->kd * derivative,
                  pid->output_min,
                  pid->output_max);
}

