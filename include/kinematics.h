#ifndef KINEMATICS_H
#define KINEMATICS_H

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    float linear_mps;
    float angular_rps;
} odometry_t;

void kinematics_inverse(float linear_mps,
                        float angular_rps,
                        float wheel_track_m,
                        float *left_mps,
                        float *right_mps);
void kinematics_forward(float left_mps,
                        float right_mps,
                        float wheel_track_m,
                        float *linear_mps,
                        float *angular_rps);
void kinematics_integrate(odometry_t *odom,
                          float left_distance_m,
                          float right_distance_m,
                          float dt_s,
                          float wheel_track_m);
float kinematics_wrap_pi(float angle_rad);

#endif /* KINEMATICS_H */
