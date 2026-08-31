#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float integral;
    float previous_error;
    float integral_limit;
    float output_min;
    float output_max;
} pid_controller_t;

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_min,
              float output_max);
void pid_reset(pid_controller_t *pid);
void pid_set_target(pid_controller_t *pid, float target);

/* 固定周期离散 PID；积分采用旧 ESP32 的“每周期累加误差”语义。 */
float pid_update(pid_controller_t *pid, float measurement);

#endif /* PID_H */

