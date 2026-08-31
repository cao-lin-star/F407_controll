#include "kinematics.h"

#include <math.h>

#define PI_F 3.14159265358979323846f

void kinematics_inverse(float linear_mps,
                        float angular_rps,
                        float wheel_track_m,
                        float *left_mps,
                        float *right_mps)
{
    *left_mps = linear_mps - angular_rps * wheel_track_m * 0.5f;
    *right_mps = linear_mps + angular_rps * wheel_track_m * 0.5f;
}

void kinematics_forward(float left_mps,
                        float right_mps,
                        float wheel_track_m,
                        float *linear_mps,
                        float *angular_rps)
{
    *linear_mps = (left_mps + right_mps) * 0.5f;
    *angular_rps = (right_mps - left_mps) / wheel_track_m;
}

float kinematics_wrap_pi(float angle_rad)
{
    while (angle_rad > PI_F) {
        angle_rad -= 2.0f * PI_F;
    }
    while (angle_rad < -PI_F) {
        angle_rad += 2.0f * PI_F;
    }
    return angle_rad;
}

void kinematics_integrate(odometry_t *odom,
                          float left_distance_m,
                          float right_distance_m,
                          float dt_s,
                          float wheel_track_m)
{
    const float center_distance = (left_distance_m + right_distance_m) * 0.5f;
    const float delta_yaw = (right_distance_m - left_distance_m) / wheel_track_m;
    const float midpoint_yaw = odom->yaw_rad + delta_yaw * 0.5f;

    if (dt_s > 0.0f) {
        odom->linear_mps = center_distance / dt_s;
        odom->angular_rps = delta_yaw / dt_s;
    } else {
        odom->linear_mps = 0.0f;
        odom->angular_rps = 0.0f;
    }

    /* 中点法比旧工程用更新后航向积分的位置误差更小。 */
    odom->x_m += center_distance * cosf(midpoint_yaw);
    odom->y_m += center_distance * sinf(midpoint_yaw);
    odom->yaw_rad = kinematics_wrap_pi(odom->yaw_rad + delta_yaw);
}
