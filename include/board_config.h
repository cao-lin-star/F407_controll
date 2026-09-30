#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Active target: STM32F407ZGT6. Pin assignments live in the CubeMX .ioc. */
#if defined(FOOTBATH_HOST_TEST)
/* Portable safety/geometry tests do not use peripheral registers. */
#elif defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#else
#include "stm32f1xx_hal.h" /* Legacy F103 PlatformIO target. */
#endif

#define BOARD_HSE_HZ                    25000000UL
#define BOARD_SYSCLK_HZ                168000000UL
#define CONTROL_PERIOD_MS                    10U
#define APP_CONTROL_SERVICE_PERIOD_MS          2U
#define APP_SENSOR_SERVICE_PERIOD_MS           2U
#define APP_TELEMETRY_SERVICE_PERIOD_MS       10U
#define APP_HOUSEKEEPING_PERIOD_MS            20U
#define ODOM_PERIOD_MS                       50U
#define RANGE_STATUS_PERIOD_MS               50U
#define IMU_PERIOD_MS                        20U
#define DEBUG_STATUS_PERIOD_MS              200U
#define HEARTBEAT_PERIOD_MS                1000U
#define CONTROL_SOURCE_PERIOD_MS            100U
#define CMD_VEL_TIMEOUT_MS                  500U
#define WHEEL_ACCEL_LIMIT_MPS2               0.30f
#define WHEEL_DECEL_LIMIT_MPS2               0.50f
#define CHASSIS_UART_BAUDRATE            115200U
#define DEBUG_UART_BAUDRATE              115200U
#define DEBUG_ASCII_PERIOD_MS               200U

/* PS2 wireless receiver: software SPI-like polling on PA4..PA7. */
#define PS2_REMOTE_ENABLE                     1U
#define PS2_POLL_PERIOD_MS                   25U
#define PS2_LINK_TIMEOUT_MS                 100U
#define PS2_CLOCK_HALF_PERIOD_US              4U
#define PS2_STICK_DEADBAND_RAW               15U
#define PS2_MANUAL_MAX_LINEAR_MPS          0.30f
#define PS2_MANUAL_MAX_ANGULAR_RPS         0.50f

/* Purchased drive: 24 V, 51:1 gearbox, 124 mm wheel, 17 PPR motor encoder. */
#define MOTOR_RATED_VOLTAGE_V               24.0f
#define MOTOR_GEAR_RATIO                     51.0f
#define MOTOR_ENCODER_BASE_PPR               17.0f
#define ENCODER_QUADRATURE_FACTOR             4.0f
#define ENCODER_COUNTS_PER_WHEEL_REV       3468.0f
#define WHEEL_DIAMETER_M                      0.124f
#define THEORETICAL_DISTANCE_PER_TICK_M       0.000112329149f

/* Measured nominal geometry; loaded rolling calibration is still required. */
#define WHEEL_TRACK_M                         0.330f
#define LEFT_DISTANCE_PER_TICK_M              THEORETICAL_DISTANCE_PER_TICK_M
#define RIGHT_DISTANCE_PER_TICK_M             THEORETICAL_DISTANCE_PER_TICK_M

/* 24.06 V loaded-floor final tuning, validated at 0.05..0.20 m/s. */
#define LEFT_PID_KP                           0.04f
#define LEFT_PID_KI                           0.002f
#define LEFT_PID_KD                           0.0f
#define RIGHT_PID_KP                          0.04f
#define RIGHT_PID_KI                          0.002f
#define RIGHT_PID_KD                          0.0f
#define PID_INTEGRAL_LIMIT_MM_S            10000.0f
#define MOTOR_OUTPUT_LIMIT_PERCENT           35.0f
#define LEFT_BREAKAWAY_FF_PERCENT              11.0f
#define RIGHT_BREAKAWAY_FF_PERCENT             17.5f
#define LEFT_RUNNING_FF_PERCENT                10.5f
#define RIGHT_RUNNING_FF_PERCENT               15.5f

#define MAX_LINEAR_MPS                        1.00f
#define MAX_ANGULAR_RPS                       1.50f
#define MAX_WHEEL_MPS                         1.00f
#define ENCODER_MAX_DELTA_PER_PERIOD           300
#define ENCODER_STALL_COMMAND_MM_S             50.0f
#define ENCODER_STALL_PWM_PERCENT              25.0f
#define ENCODER_STALL_TIMEOUT_MS              750U
#define ENCODER_RECOVERY_STABLE_MS            500U
#define ENCODER_RECOVERY_MAX_DELTA_PER_PERIOD    2U
#define TRANSIENT_FAULT_CLEAR_MS              1000U

/* Change only after a wheels-off-ground direction test. */
#define LEFT_MOTOR_COMMAND_SIGN                  1
#define RIGHT_MOTOR_COMMAND_SIGN                -1
#define LEFT_ENCODER_SIGN                       -1
#define RIGHT_ENCODER_SIGN                       1
/* Enable only after IBT-2 braking current/backfeed is bench-verified. */
#define MOTOR_STOP_BRAKE                         0
#define MOTOR_PWM_FREQUENCY_HZ                20000UL
#define MOTOR_PWM_PERIOD_COUNTS                 8399UL
#define ENCODER_INPUT_FILTER                       6U

/* Purchased Nooploop TOFSense-M S, UART active-output protocol. */
#define TOF_PROTOCOL_DISABLED                     0U
#define TOF_PROTOCOL_TOFSENSE_M                   2U
#define TOF_PROTOCOL_MODE        TOF_PROTOCOL_TOFSENSE_M
#define TOFSENSE_UART_BAUDRATE                921600U
#define TOFSENSE_FRAME_MAX_BYTES                 400U
#define TOFSENSE_MIN_VALID_ZONES_4X4               8U
#define TOFSENSE_MIN_VALID_ZONES_8X8              32U
#define TOFSENSE_RX_RING_SIZE                   1024U
#define TOFSENSE_MIN_DISTANCE_M                 0.015f
#define TOFSENSE_MAX_DISTANCE_M                 4.000f

/* Purchased CS100A/new HC-SR04, trigger/echo interface. */
#define ULTRASONIC_MODE_DISABLED                  0U
#define ULTRASONIC_MODE_TRIGGER_ECHO              1U
#define ULTRASONIC_PROTOCOL_MODE ULTRASONIC_MODE_TRIGGER_ECHO /* PB6 / TIM4_CH1. */
#define ULTRASONIC_TRIGGER_PERIOD_MS              80U
#define ULTRASONIC_ECHO_TIMEOUT_MS                70U
#define ULTRASONIC_MIN_DISTANCE_M               0.02f
#define ULTRASONIC_MAX_DISTANCE_M               4.00f

/*
 * Drivers and telemetry are active. The confirmed front ultrasonic is an
 * independent forward-motion stop source. Cliff recovery permits only a
 * zero-speed acknowledgement followed by reverse motion.
 */
#define FRONT_OBSTACLE_SAFETY_ENABLE              1U
#define IR_OBSTACLE_SAFETY_ENABLE                 0U
#define FRONT_OBSTACLE_STOP_DISTANCE_M          0.20f
#define FRONT_OBSTACLE_CLEAR_DISTANCE_M         0.28f
/* Side sonar: preserve 80 ms front/left/front/right polling. */
#define SIDE_ULTRASONIC_SAFETY_ENABLE              1U
#define SIDE_ULTRASONIC_TIMEOUT_MS              600U
#define SIDE_ULTRASONIC_STOP_DISTANCE_M          0.12f
#define CLIFF_SAFETY_ENABLE                       1U
#define CLIFF_LEFT_GROUND_BASELINE_M             0.155f
#define CLIFF_RIGHT_GROUND_BASELINE_M            0.160f
#define CLIFF_STOP_DISTANCE_M                    0.28f /* Absolute probe distance, both sides. */
#define CLIFF_CLEAR_DISTANCE_M                   0.26f /* 2 cm hysteresis. */
#define CLIFF_FAIL_SAFE_ON_TIMEOUT                 1U
#define SENSOR_DATA_TIMEOUT_MS                   250U

/* The purchased GY-521 batch may report WHO_AM_I 0x68 or clone ID 0x70. */
#define MPU6050_I2C_ADDRESS_7BIT                0x68U
#define MPU6050_WHO_AM_I_ORIGINAL               0x68U
#define MPU6050_WHO_AM_I_NEW_BATCH              0x70U
#define MPU6050_SAMPLE_RATE_DIVIDER               19U /* 1 kHz/(19+1)=50 Hz */

#define DEBUG_STATUS_ENABLE                        1U

#endif /* BOARD_CONFIG_H */
