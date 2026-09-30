#include "sensor_hub.h"

#include <math.h>
#include <string.h>

#include "board_config.h"
#include "ultrasonic_measure.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_WHO_AM_I     0x75U
#define MPU6050_ADDRESS          (MPU6050_I2C_ADDRESS_7BIT << 1U)
#define MPU6050_BOOT_DELAY_MS    100U
#define MPU6050_ID_RETRY_COUNT     3U
#define MPU6050_ID_RETRY_DELAY_MS 20U
#define MPU6050_CALIBRATION_SAMPLES 100U
#define MPU6050_CALIBRATION_MIN_OK   80U
#define MPU6050_CALIBRATION_DELAY_MS 10U

#define TOFSENSE_FRAME_HEADER       0x57U
#define TOFSENSE_OUTPUT_FUNCTION    0x01U
#define TOFSENSE_ZONES_4X4          0x10U
#define TOFSENSE_ZONES_8X8          0x40U
#define TOFSENSE_FRAME_4X4_BYTES     112U
#define TOFSENSE_FRAME_8X8_BYTES     400U
#define TOFSENSE_ZONE_DATA_OFFSET      9U
#define TOFSENSE_ZONE_DATA_BYTES       6U
#define TOFSENSE_STATUS_VALID          0U
#define TOFSENSE_RAW_MIN           15000UL
#define TOFSENSE_RAW_MAX         4000000UL

typedef struct {
    uint8_t frame[TOFSENSE_FRAME_MAX_BYTES];
    uint16_t count;
    uint16_t expected_length;
    uint8_t rx_byte;
    uint32_t last_update_ms;
    uint32_t last_frame_ms;
    uint32_t checksum_errors;
    uint32_t format_errors;
} tofsense_parser_t;

typedef struct {
    uint8_t data[TOFSENSE_RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t overflows;
    volatile uint32_t uart_errors;
    uint32_t handled_transport_errors;
} tofsense_rx_ring_t;

typedef char tofsense_rx_ring_size_must_be_power_of_two[
    ((TOFSENSE_RX_RING_SIZE & (TOFSENSE_RX_RING_SIZE - 1U)) == 0U) ? 1 : -1];

static sensor_snapshot_t snapshot;
#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
static tofsense_parser_t tof_left;
static tofsense_parser_t tof_right;
static tofsense_rx_ring_t tof_left_rx;
static tofsense_rx_ring_t tof_right_rx;
#endif
static uint32_t next_imu_ms;
#if ULTRASONIC_PROTOCOL_MODE == ULTRASONIC_MODE_TRIGGER_ECHO
static uint32_t next_ultrasonic_trigger_ms;
static const uint32_t us_channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static const uint32_t us_active_channels[3] = {HAL_TIM_ACTIVE_CHANNEL_1, HAL_TIM_ACTIVE_CHANNEL_2, HAL_TIM_ACTIVE_CHANNEL_3};
static const uint16_t us_trig_pins[3] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2};
static const uint8_t us_schedule[4] = {0,1,0,2};
static volatile uint8_t us_index;
static uint8_t us_slot;
static uint32_t us_stamp[3];
static volatile uint16_t ultrasonic_rising_us;
static volatile uint32_t ultrasonic_rising_ms;
static volatile uint8_t ultrasonic_long_pulse;
static volatile uint16_t ultrasonic_captured_pulse_us;
static uint32_t ultrasonic_last_update_ms;
static uint32_t ultrasonic_measurement_started_ms;
static volatile uint8_t ultrasonic_measurement_pending;
static volatile uint8_t ultrasonic_wait_falling;
static volatile uint8_t ultrasonic_pulse_ready;
#endif
static uint8_t imu_present;
static int32_t gyro_bias_x_raw;
static int32_t gyro_bias_y_raw;
static int32_t gyro_bias_z_raw;
static float accel_scale_correction = 1.0f;

static uint32_t enter_critical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void leave_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static int16_t read_i16_be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static HAL_StatusTypeDef mpu_write_u8(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             MPU6050_ADDRESS,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             10U);
}

static uint8_t mpu_id_supported(uint8_t who_am_i)
{
    return (uint8_t)(who_am_i == MPU6050_WHO_AM_I_ORIGINAL
                     || who_am_i == MPU6050_WHO_AM_I_NEW_BATCH);
}

static uint8_t mpu6050_read_raw(uint8_t raw[14], uint32_t timeout_ms)
{
    return (uint8_t)(HAL_I2C_Mem_Read(&hi2c1,
                                      MPU6050_ADDRESS,
                                      MPU6050_REG_ACCEL_XOUT_H,
                                      I2C_MEMADD_SIZE_8BIT,
                                      raw,
                                      14U,
                                      timeout_ms) == HAL_OK);
}

static uint8_t mpu6050_calibrate_stationary(void)
{
    uint8_t raw[14];
    uint16_t sample;
    uint16_t valid = 0U;
    int64_t gyro_x_sum = 0;
    int64_t gyro_y_sum = 0;
    int64_t gyro_z_sum = 0;
    float accel_norm_sum = 0.0f;

    HAL_Delay(100U);
    for (sample = 0U; sample < MPU6050_CALIBRATION_SAMPLES; ++sample) {
        if (mpu6050_read_raw(raw, 10U) != 0U) {
            const float ax = (float)read_i16_be(&raw[0]);
            const float ay = (float)read_i16_be(&raw[2]);
            const float az = (float)read_i16_be(&raw[4]);
            accel_norm_sum += sqrtf(ax * ax + ay * ay + az * az);
            gyro_x_sum += read_i16_be(&raw[8]);
            gyro_y_sum += read_i16_be(&raw[10]);
            gyro_z_sum += read_i16_be(&raw[12]);
            ++valid;
        }
        HAL_Delay(MPU6050_CALIBRATION_DELAY_MS);
    }
    if (valid < MPU6050_CALIBRATION_MIN_OK) {
        return 0U;
    }

    gyro_bias_x_raw = (int32_t)(gyro_x_sum / (int64_t)valid);
    gyro_bias_y_raw = (int32_t)(gyro_y_sum / (int64_t)valid);
    gyro_bias_z_raw = (int32_t)(gyro_z_sum / (int64_t)valid);
    accel_scale_correction = 8192.0f / (accel_norm_sum / (float)valid);
    if (accel_scale_correction < 0.8f || accel_scale_correction > 1.2f) {
        accel_scale_correction = 1.0f;
        return 0U;
    }
    return 1U;
}

static void mpu6050_i2c_bus_recover(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t pulse;

    (void)HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = MPU6050_SCL_Pin | MPU6050_SDA_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, MPU6050_SCL_Pin | MPU6050_SDA_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);
    for (pulse = 0U; pulse < 9U; ++pulse) {
        HAL_GPIO_WritePin(GPIOB, MPU6050_SCL_Pin, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, MPU6050_SCL_Pin, GPIO_PIN_SET);
        HAL_Delay(1U);
    }
    /* Generate a STOP condition while SCL is high. */
    HAL_GPIO_WritePin(GPIOB, MPU6050_SDA_Pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, MPU6050_SCL_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, MPU6050_SDA_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);

    HAL_GPIO_DeInit(GPIOB, MPU6050_SCL_Pin | MPU6050_SDA_Pin);
    MX_I2C1_Init();
}

static void mpu6050_init(void)
{
    uint8_t who_am_i = 0U;
    uint8_t retry;

    /* sensor_hub_init() runs before the 10 ms control scheduler starts, so
     * this boot-only delay cannot cause a control overrun. Some MPU6050
     * clones do not acknowledge immediately after the board rails rise. */
    mpu6050_i2c_bus_recover();
    HAL_Delay(MPU6050_BOOT_DELAY_MS);
    for (retry = 0U; retry < MPU6050_ID_RETRY_COUNT; ++retry) {
        if (HAL_I2C_Mem_Read(&hi2c1,
                             MPU6050_ADDRESS,
                             MPU6050_REG_WHO_AM_I,
                             I2C_MEMADD_SIZE_8BIT,
                             &who_am_i,
                             1U,
                             20U) == HAL_OK
            && mpu_id_supported(who_am_i) != 0U) {
            break;
        }
        HAL_Delay(MPU6050_ID_RETRY_DELAY_MS);
    }
    if (retry >= MPU6050_ID_RETRY_COUNT) {
        snapshot.sensor_fault_flags |= SENSOR_FAULT_MPU6050;
        imu_present = 0U;
        return;
    }

    snapshot.imu_who_am_i = who_am_i;
    if (mpu_write_u8(MPU6050_REG_PWR_MGMT_1, 0x80U) != HAL_OK) {
        snapshot.sensor_fault_flags |= SENSOR_FAULT_MPU6050;
        imu_present = 0U;
        return;
    }
    HAL_Delay(100U);

    if (mpu_write_u8(MPU6050_REG_PWR_MGMT_1, 0x01U) != HAL_OK
        || mpu_write_u8(MPU6050_REG_SMPLRT_DIV,
                        MPU6050_SAMPLE_RATE_DIVIDER) != HAL_OK
        || mpu_write_u8(MPU6050_REG_CONFIG, 0x03U) != HAL_OK
        || mpu_write_u8(MPU6050_REG_GYRO_CONFIG, 0x08U) != HAL_OK
        || mpu_write_u8(MPU6050_REG_ACCEL_CONFIG, 0x08U) != HAL_OK) {
        snapshot.sensor_fault_flags |= SENSOR_FAULT_MPU6050;
        imu_present = 0U;
        return;
    }

    snapshot.sensor_fault_flags &= (uint16_t)~SENSOR_FAULT_MPU6050;
    imu_present = 1U;
    if (mpu6050_calibrate_stationary() == 0U) {
        snapshot.sensor_fault_flags |= SENSOR_FAULT_MPU6050;
        imu_present = 0U;
    }
}

static void mpu6050_sample(uint32_t now_ms)
{
    uint8_t raw[14];
    const float accel_scale = 9.80665f / 8192.0f;   /* +/-4 g */
    const float gyro_scale = 0.01745329252f / 65.5f; /* +/-500 dps */

    if (imu_present == 0U) {
        return;
    }
    if (mpu6050_read_raw(raw, 5U) == 0U) {
        snapshot.valid_flags &= (uint16_t)~RANGE_VALID_IMU;
        snapshot.sensor_fault_flags |= SENSOR_FAULT_MPU6050;
        return;
    }

    snapshot.accel_x_mps2 = (float)read_i16_be(&raw[0]) * accel_scale
                          * accel_scale_correction;
    snapshot.accel_y_mps2 = (float)read_i16_be(&raw[2]) * accel_scale
                          * accel_scale_correction;
    snapshot.accel_z_mps2 = (float)read_i16_be(&raw[4]) * accel_scale
                          * accel_scale_correction;
    snapshot.gyro_x_rps = (float)(read_i16_be(&raw[8]) - gyro_bias_x_raw)
                        * gyro_scale;
    snapshot.gyro_y_rps = (float)(read_i16_be(&raw[10]) - gyro_bias_y_raw)
                        * gyro_scale;
    snapshot.gyro_z_rps = (float)(read_i16_be(&raw[12]) - gyro_bias_z_raw)
                        * gyro_scale;
    snapshot.imu_stamp_ms = now_ms;
    snapshot.valid_flags |= RANGE_VALID_IMU;
    snapshot.sensor_fault_flags &= (uint16_t)~SENSOR_FAULT_MPU6050;
}

#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
static uint32_t read_u24_le(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8U)
         | ((uint32_t)data[2] << 16U);
}

static void sort_u32(uint32_t *values, uint8_t count)
{
    uint8_t i;

    for (i = 1U; i < count; ++i) {
        const uint32_t value = values[i];
        uint8_t j = i;
        while (j > 0U && values[j - 1U] > value) {
            values[j] = values[j - 1U];
            --j;
        }
        values[j] = value;
    }
}

static void tofsense_restart(tofsense_parser_t *parser, uint8_t byte)
{
    parser->count = 0U;
    parser->expected_length = 0U;
    if (byte == TOFSENSE_FRAME_HEADER) {
        parser->frame[0] = byte;
        parser->count = 1U;
    }
}

static uint8_t tofsense_checksum_valid(const tofsense_parser_t *parser)
{
    uint16_t i;
    uint8_t sum = 0U;

    for (i = 0U; i < (uint16_t)(parser->expected_length - 1U); ++i) {
        sum = (uint8_t)(sum + parser->frame[i]);
    }
    return (uint8_t)(sum == parser->frame[parser->expected_length - 1U]);
}

static void tofsense_accept_frame(tofsense_parser_t *parser,
                                  float *distance_m,
                                  uint8_t *valid_zones_out,
                                  uint8_t *zone_count_out,
                                  uint32_t *frame_count_out,
                                  uint16_t valid_bit,
                                  uint16_t fault_bit)
{
    uint32_t valid_distance_raw[64];
    uint32_t median_raw;
    uint8_t valid_count = 0U;
    uint8_t zone_count = parser->frame[8];
    uint8_t minimum_valid;
    uint8_t i;

    ++(*frame_count_out);
    parser->last_frame_ms = HAL_GetTick();
    *zone_count_out = zone_count;
    minimum_valid = zone_count == TOFSENSE_ZONES_4X4
                  ? TOFSENSE_MIN_VALID_ZONES_4X4
                  : TOFSENSE_MIN_VALID_ZONES_8X8;

    for (i = 0U; i < zone_count; ++i) {
        const uint16_t offset = (uint16_t)(TOFSENSE_ZONE_DATA_OFFSET
                                + (uint16_t)i * TOFSENSE_ZONE_DATA_BYTES);
        const uint32_t raw_distance = read_u24_le(&parser->frame[offset]);
        const uint8_t distance_status = parser->frame[offset + 3U];

        if (distance_status == TOFSENSE_STATUS_VALID
            && raw_distance >= TOFSENSE_RAW_MIN
            && raw_distance <= TOFSENSE_RAW_MAX) {
            valid_distance_raw[valid_count++] = raw_distance;
        }
    }

    *valid_zones_out = valid_count;
    if (valid_count < minimum_valid) {
        snapshot.valid_flags &= (uint16_t)~valid_bit;
        snapshot.sensor_fault_flags |= fault_bit;
        return;
    }

    sort_u32(valid_distance_raw, valid_count);
    if ((valid_count & 1U) != 0U) {
        median_raw = valid_distance_raw[valid_count / 2U];
    } else {
        median_raw = (valid_distance_raw[valid_count / 2U - 1U]
                      + valid_distance_raw[valid_count / 2U]) / 2U;
    }

    *distance_m = (float)median_raw * 0.000001f;
    parser->last_update_ms = HAL_GetTick();
    snapshot.valid_flags |= valid_bit;
    snapshot.sensor_fault_flags &= (uint16_t)~fault_bit;
}

static void tofsense_feed(tofsense_parser_t *parser,
                          uint8_t byte,
                          float *distance_m,
                          uint8_t *valid_zones_out,
                          uint8_t *zone_count_out,
                          uint32_t *frame_count_out,
                          uint16_t valid_bit,
                          uint16_t fault_bit)
{
    if (parser->count == 0U) {
        if (byte == TOFSENSE_FRAME_HEADER) {
            parser->frame[0] = byte;
            parser->count = 1U;
        }
        return;
    }

    if (parser->count >= TOFSENSE_FRAME_MAX_BYTES) {
        ++parser->format_errors;
        snapshot.sensor_fault_flags |= fault_bit;
        tofsense_restart(parser, byte);
        return;
    }

    parser->frame[parser->count++] = byte;
    if (parser->count == 2U
        && parser->frame[1] != TOFSENSE_OUTPUT_FUNCTION) {
        ++parser->format_errors;
        snapshot.sensor_fault_flags |= fault_bit;
        tofsense_restart(parser, byte);
        return;
    }

    if (parser->count == 9U) {
        if (parser->frame[8] == TOFSENSE_ZONES_4X4) {
            parser->expected_length = TOFSENSE_FRAME_4X4_BYTES;
        } else if (parser->frame[8] == TOFSENSE_ZONES_8X8) {
            parser->expected_length = TOFSENSE_FRAME_8X8_BYTES;
        } else {
            ++parser->format_errors;
            snapshot.sensor_fault_flags |= fault_bit;
            tofsense_restart(parser, byte);
            return;
        }
    }

    if (parser->expected_length == 0U
        || parser->count < parser->expected_length) {
        return;
    }

    if (tofsense_checksum_valid(parser) != 0U) {
        tofsense_accept_frame(parser,
                              distance_m,
                              valid_zones_out,
                              zone_count_out,
                              frame_count_out,
                              valid_bit,
                              fault_bit);
    } else {
        ++parser->checksum_errors;
        snapshot.sensor_fault_flags |= fault_bit;
    }
    parser->count = 0U;
    parser->expected_length = 0U;
}
static void tofsense_rx_push(tofsense_rx_ring_t *ring, uint8_t byte)
{
    const uint16_t next_head = (uint16_t)((ring->head + 1U)
                               & (TOFSENSE_RX_RING_SIZE - 1U));

    if (next_head == ring->tail) {
        ++ring->overflows;
        return;
    }
    ring->data[ring->head] = byte;
    ring->head = next_head;
}

static uint8_t tofsense_rx_pop(tofsense_rx_ring_t *ring, uint8_t *byte)
{
    const uint16_t tail = ring->tail;

    if (tail == ring->head) {
        return 0U;
    }
    *byte = ring->data[tail];
    ring->tail = (uint16_t)((tail + 1U) & (TOFSENSE_RX_RING_SIZE - 1U));
    return 1U;
}

static void tofsense_ensure_rx_armed(UART_HandleTypeDef *uart,
                                     tofsense_parser_t *parser,
                                     tofsense_rx_ring_t *ring)
{
    if (uart->RxState == HAL_UART_STATE_READY
        && HAL_UART_Receive_IT(uart, &parser->rx_byte, 1U) != HAL_OK) {
        ++ring->uart_errors;
    }
}

static void tofsense_process_rx(tofsense_parser_t *parser,
                                tofsense_rx_ring_t *ring,
                                UART_HandleTypeDef *uart,
                                float *distance_m,
                                uint8_t *valid_zones_out,
                                uint8_t *zone_count_out,
                                uint32_t *frame_count_out,
                                uint16_t valid_bit,
                                uint16_t fault_bit)
{
    uint8_t byte;
    const uint32_t transport_errors = ring->overflows + ring->uart_errors;

    tofsense_ensure_rx_armed(uart, parser, ring);
    if (transport_errors != ring->handled_transport_errors) {
        ring->handled_transport_errors = transport_errors;
        ring->tail = ring->head;
        tofsense_restart(parser, 0U);
        snapshot.sensor_fault_flags |= fault_bit;
    }
    while (tofsense_rx_pop(ring, &byte) != 0U) {
        tofsense_feed(parser,
                      byte,
                      distance_m,
                      valid_zones_out,
                      zone_count_out,
                      frame_count_out,
                      valid_bit,
                      fault_bit);
    }
}
#endif

static uint16_t update_cliff_flag(uint16_t flags,
                                  uint16_t flag,
                                  uint16_t valid_bit,
                                  float distance_m,
                                  float ground_baseline_m)
{
    if (ground_baseline_m <= 0.0f) {
        return (uint16_t)(flags & (uint16_t)~flag);
    }
    if ((snapshot.valid_flags & valid_bit) == 0U) {
#if CLIFF_SAFETY_ENABLE && CLIFF_FAIL_SAFE_ON_TIMEOUT
        return (uint16_t)(flags | flag);
#else
        return (uint16_t)(flags & (uint16_t)~flag);
#endif
    }
    if (distance_m >= CLIFF_STOP_DISTANCE_M) {
        flags |= flag;
    } else if (distance_m <= CLIFF_CLEAR_DISTANCE_M) {
        flags &= (uint16_t)~flag;
    }
    return flags;
}

static void update_obstacle_flags(void)
{
    uint16_t flags = snapshot.obstacle_flags;

    flags = update_cliff_flag(flags,
                              OBSTACLE_CLIFF_LEFT,
                              RANGE_VALID_TOF_LEFT,
                              snapshot.tof_left_m,
                              CLIFF_LEFT_GROUND_BASELINE_M);
    flags = update_cliff_flag(flags,
                              OBSTACLE_CLIFF_RIGHT,
                              RANGE_VALID_TOF_RIGHT,
                              snapshot.tof_right_m,
                              CLIFF_RIGHT_GROUND_BASELINE_M);

    if ((snapshot.valid_flags & RANGE_VALID_ULTRASONIC) != 0U) {
        if (snapshot.ultrasonic_m <= FRONT_OBSTACLE_STOP_DISTANCE_M) {
            flags |= OBSTACLE_ULTRASONIC;
        } else if (snapshot.ultrasonic_m >= FRONT_OBSTACLE_CLEAR_DISTANCE_M) {
            flags &= (uint16_t)~OBSTACLE_ULTRASONIC;
        }
    } else {
        flags &= (uint16_t)~OBSTACLE_ULTRASONIC;
    }

    if (HAL_GPIO_ReadPin(IR_LEFT_N_GPIO_Port, IR_LEFT_N_Pin) == GPIO_PIN_RESET) {
        flags |= OBSTACLE_IR_LEFT;
    } else {
        flags &= (uint16_t)~OBSTACLE_IR_LEFT;
    }
    if (HAL_GPIO_ReadPin(IR_RIGHT_N_GPIO_Port, IR_RIGHT_N_Pin) == GPIO_PIN_RESET) {
        flags |= OBSTACLE_IR_RIGHT;
    } else {
        flags &= (uint16_t)~OBSTACLE_IR_RIGHT;
    }
    snapshot.obstacle_flags = flags;
}

#if ULTRASONIC_PROTOCOL_MODE == ULTRASONIC_MODE_TRIGGER_ECHO
static void ultrasonic_publish_clear(uint32_t now_ms)
{
    snapshot.ultrasonic_three_m[us_index] = ULTRASONIC_MAX_DISTANCE_M;
    snapshot.ultrasonic_status[us_index] = 2U;
    ++snapshot.ultrasonic_sequence[us_index];
    us_stamp[us_index] = now_ms;
    if (us_index != 0U) return;
    /* HC-SR04-class modules report an open/out-of-range scene as no echo.
     * Publish the finite maximum range so the RK costmap can ray-clear to the
     * sensor limit. The UART protocol deliberately rejects NaN/Inf. */
    snapshot.ultrasonic_m = ULTRASONIC_MAX_DISTANCE_M;
    ultrasonic_last_update_ms = now_ms;
    snapshot.valid_flags |= RANGE_VALID_ULTRASONIC;
    snapshot.sensor_fault_flags &= (uint16_t)~SENSOR_FAULT_ULTRASONIC;
}

static void ultrasonic_publish_fault(void)
{
    snapshot.ultrasonic_status[us_index] = 3U;
    ++snapshot.ultrasonic_sequence[us_index];
    us_stamp[us_index] = HAL_GetTick();
    if (us_index != 0U) return;
    snapshot.valid_flags &= (uint16_t)~RANGE_VALID_ULTRASONIC;
    snapshot.sensor_fault_flags |= SENSOR_FAULT_ULTRASONIC;
}

static void ultrasonic_start_measurement(uint32_t now_ms)
{
    uint32_t primask;
    uint16_t trigger_started_us;

    GPIO_TypeDef *echo_port = us_index == 0U ? GPIOB : GPIOD;
    const uint16_t echo_pin = us_index == 0U ? GPIO_PIN_6 : (us_index == 1U ? GPIO_PIN_13 : GPIO_PIN_14);
    if (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET) {
        ultrasonic_publish_fault();
        return; /* Stuck high before trigger is not open space. */
    }

    primask = enter_critical();
    ultrasonic_measurement_pending = 1U;
    ultrasonic_wait_falling = 0U;
    ultrasonic_pulse_ready = 0U;
    ultrasonic_long_pulse = 0U;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4,
                                  us_channels[us_index],
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    leave_critical(primask);

    ultrasonic_measurement_started_ms = now_ms;
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port,
                      us_trig_pins[us_index],
                      GPIO_PIN_SET);
    trigger_started_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    /* The sensor service runs every 2 ms, so deferring this edge to the next
     * service would create a millisecond-long trigger. A bounded 12 us wait
     * every 80 ms keeps TRIG inside the module specification without moving
     * timing work into an interrupt. */
    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim4)
                      - trigger_started_us) < 12U) {
        /* bounded hardware-timer wait */
    }
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port,
                      us_trig_pins[us_index],
                      GPIO_PIN_RESET);
}

static void ultrasonic_process_measurement(uint32_t now_ms)
{
    uint32_t primask;
    uint16_t pulse_us = 0U;
    uint8_t pulse_ready;
    uint8_t saw_rising_edge;
    float distance_m;

    primask = enter_critical();
    pulse_ready = ultrasonic_pulse_ready;
    if (pulse_ready != 0U) {
        pulse_us = ultrasonic_captured_pulse_us;
        ultrasonic_pulse_ready = 0U;
        ultrasonic_measurement_pending = 0U;
    }
    leave_critical(primask);

    if (pulse_ready != 0U) {
        /* CS100A emits ~66ms for no target, exceeding TIM4's 65536us wrap. */
        distance_m = ultrasonic_pulse_distance(pulse_us, ultrasonic_long_pulse ? 60U : 0U);
        if (distance_m < ULTRASONIC_MIN_DISTANCE_M) {
            /* A zero/very short pulse is electrical noise, not open space. */
            ultrasonic_publish_fault();
        } else if (distance_m > ULTRASONIC_MAX_DISTANCE_M) {
            ultrasonic_publish_clear(now_ms);
        } else {
            snapshot.ultrasonic_three_m[us_index] = distance_m;
            snapshot.ultrasonic_status[us_index] = 1U;
            ++snapshot.ultrasonic_sequence[us_index];
            us_stamp[us_index] = now_ms;
            if (us_index == 0U) {
                snapshot.ultrasonic_m = distance_m;
                ultrasonic_last_update_ms = now_ms;
                snapshot.valid_flags |= RANGE_VALID_ULTRASONIC;
                snapshot.sensor_fault_flags &= (uint16_t)~SENSOR_FAULT_ULTRASONIC;
            }
        }
        return;
    }

    if (ultrasonic_measurement_pending == 0U
        || (uint32_t)(now_ms - ultrasonic_measurement_started_ms)
           <= ULTRASONIC_ECHO_TIMEOUT_MS) {
        return;
    }

    primask = enter_critical();
    saw_rising_edge = ultrasonic_wait_falling;
    ultrasonic_measurement_pending = 0U;
    ultrasonic_wait_falling = 0U;
    ultrasonic_pulse_ready = 0U;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4,
                                  us_channels[us_index],
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    leave_critical(primask);

    if (saw_rising_edge != 0U) {
        /* ECHO became high but never returned low: malformed/stuck signal. */
        ultrasonic_publish_fault();
    } else {
        /* No echo at all is the normal representation of an open scene. */
        ultrasonic_publish_clear(now_ms);
    }
}
#endif
void sensor_hub_init(void)
{
    memset(&snapshot, 0, sizeof(snapshot));
#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
    memset(&tof_left, 0, sizeof(tof_left));
    memset(&tof_right, 0, sizeof(tof_right));
    memset(&tof_left_rx, 0, sizeof(tof_left_rx));
    memset(&tof_right_rx, 0, sizeof(tof_right_rx));
#endif
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port,
                      us_trig_pins[us_index],
                      GPIO_PIN_RESET);
    mpu6050_init();

#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
    snapshot.sensor_fault_flags |= SENSOR_FAULT_TOF_LEFT
                                 | SENSOR_FAULT_TOF_RIGHT;
    tofsense_ensure_rx_armed(&huart2, &tof_left, &tof_left_rx);
    tofsense_ensure_rx_armed(&huart3, &tof_right, &tof_right_rx);
#else
    snapshot.sensor_fault_flags |= SENSOR_FAULT_TOF_UNVERIFIED;
#endif

#if ULTRASONIC_PROTOCOL_MODE == ULTRASONIC_MODE_TRIGGER_ECHO
    snapshot.sensor_fault_flags |= SENSOR_FAULT_ULTRASONIC;
    ultrasonic_measurement_pending = 0U;
    ultrasonic_wait_falling = 0U;
    ultrasonic_pulse_ready = 0U;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4,
                                  us_channels[us_index],
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    (void)HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
    (void)HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
    (void)HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
    us_index = 0U;
    us_slot = 0U;
    memset(us_stamp, 0, sizeof(us_stamp));
#else
    snapshot.sensor_fault_flags |= SENSOR_FAULT_US_UNVERIFIED;
#endif

    next_imu_ms = HAL_GetTick();
#if ULTRASONIC_PROTOCOL_MODE == ULTRASONIC_MODE_TRIGGER_ECHO
    next_ultrasonic_trigger_ms = HAL_GetTick() + 100U;
#endif
}

void sensor_hub_process(uint32_t now_ms)
{
#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
    tofsense_process_rx(&tof_left,
                        &tof_left_rx,
                        &huart2,
                        &snapshot.tof_left_m,
                        &snapshot.tof_left_valid_zones,
                        &snapshot.tof_left_zone_count,
                        &snapshot.tof_left_frames,
                        RANGE_VALID_TOF_LEFT,
                        SENSOR_FAULT_TOF_LEFT);
    tofsense_process_rx(&tof_right,
                        &tof_right_rx,
                        &huart3,
                        &snapshot.tof_right_m,
                        &snapshot.tof_right_valid_zones,
                        &snapshot.tof_right_zone_count,
                        &snapshot.tof_right_frames,
                        RANGE_VALID_TOF_RIGHT,
                        SENSOR_FAULT_TOF_RIGHT);
#endif

    if ((int32_t)(now_ms - next_imu_ms) >= 0) {
        next_imu_ms += IMU_PERIOD_MS;
        if ((int32_t)(now_ms - next_imu_ms) >= 0) {
            next_imu_ms = now_ms + IMU_PERIOD_MS;
        }
        mpu6050_sample(now_ms);
    }

#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
    const uint32_t tof_now_ms = HAL_GetTick();
    snapshot.valid_flags &= (uint16_t)~(RANGE_TOF_LEFT_FRAME_FRESH | RANGE_TOF_RIGHT_FRAME_FRESH);
    if (snapshot.tof_left_frames > 0U && tof_now_ms - tof_left.last_frame_ms <= SENSOR_DATA_TIMEOUT_MS)
        snapshot.valid_flags |= RANGE_TOF_LEFT_FRAME_FRESH;
    if (snapshot.tof_right_frames > 0U && tof_now_ms - tof_right.last_frame_ms <= SENSOR_DATA_TIMEOUT_MS)
        snapshot.valid_flags |= RANGE_TOF_RIGHT_FRAME_FRESH;
    snapshot.tof_left_age_ms = tof_now_ms - tof_left.last_update_ms;
    snapshot.tof_right_age_ms = tof_now_ms - tof_right.last_update_ms;
    if ((snapshot.valid_flags & RANGE_VALID_TOF_LEFT) != 0U
        && (uint32_t)(tof_now_ms - tof_left.last_update_ms) > SENSOR_DATA_TIMEOUT_MS) {
        snapshot.valid_flags &= (uint16_t)~RANGE_VALID_TOF_LEFT;

        snapshot.sensor_fault_flags |= SENSOR_FAULT_TOF_LEFT;
    }
    if ((snapshot.valid_flags & RANGE_VALID_TOF_RIGHT) != 0U
        && (uint32_t)(tof_now_ms - tof_right.last_update_ms) > SENSOR_DATA_TIMEOUT_MS) {
        snapshot.valid_flags &= (uint16_t)~RANGE_VALID_TOF_RIGHT;
        snapshot.sensor_fault_flags |= SENSOR_FAULT_TOF_RIGHT;
    }
#endif

#if ULTRASONIC_PROTOCOL_MODE == ULTRASONIC_MODE_TRIGGER_ECHO
    ultrasonic_process_measurement(now_ms);

    if ((int32_t)(now_ms - next_ultrasonic_trigger_ms) >= 0
        && ultrasonic_measurement_pending == 0U) {
        us_index = us_schedule[us_slot];
        us_slot = (uint8_t)((us_slot + 1U) % 4U);
        ultrasonic_start_measurement(now_ms);
        next_ultrasonic_trigger_ms = now_ms + ULTRASONIC_TRIGGER_PERIOD_MS;
    }
    if ((snapshot.valid_flags & RANGE_VALID_ULTRASONIC) != 0U
        && (uint32_t)(now_ms - ultrasonic_last_update_ms)
           > SENSOR_DATA_TIMEOUT_MS) {
        snapshot.valid_flags &= (uint16_t)~RANGE_VALID_ULTRASONIC;
        snapshot.sensor_fault_flags |= SENSOR_FAULT_ULTRASONIC;
    }
#endif

    update_obstacle_flags();
    for (unsigned i = 0; i < 3U; ++i) {
        snapshot.ultrasonic_age_ms[i] = now_ms - us_stamp[i];
        if (snapshot.ultrasonic_status[i] != 0U && snapshot.ultrasonic_age_ms[i] > 600U)
            snapshot.ultrasonic_status[i] = 3U;
    }
}

void sensor_hub_get_snapshot(sensor_snapshot_t *out)
{
    uint32_t primask;
    if (out == NULL) {
        return;
    }
    primask = enter_critical();
    *out = snapshot;
#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
    out->tof_left_rx_errors = tof_left_rx.overflows
                            + tof_left_rx.uart_errors
                            + tof_left.checksum_errors
                            + tof_left.format_errors;
    out->tof_right_rx_errors = tof_right_rx.overflows
                             + tof_right_rx.uart_errors
                             + tof_right.checksum_errors
                             + tof_right.format_errors;
#endif
    leave_critical(primask);
}

bool sensor_hub_obstacle_stop_required(void)
{
    uint16_t stop_mask = 0U;

#if FRONT_OBSTACLE_SAFETY_ENABLE
    stop_mask |= OBSTACLE_ULTRASONIC;
#endif
#if IR_OBSTACLE_SAFETY_ENABLE
    stop_mask |= OBSTACLE_IR_LEFT | OBSTACLE_IR_RIGHT;
#endif
#if CLIFF_SAFETY_ENABLE
    stop_mask |= OBSTACLE_CLIFF_LEFT | OBSTACLE_CLIFF_RIGHT;
#endif
    return (snapshot.obstacle_flags & stop_mask) != 0U;
}

void sensor_hub_on_uart_rx_complete(UART_HandleTypeDef *uart)
{
#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
    if (uart == &huart2) {
        const uint8_t byte = tof_left.rx_byte;
        tofsense_ensure_rx_armed(&huart2, &tof_left, &tof_left_rx);
        tofsense_rx_push(&tof_left_rx, byte);
    } else if (uart == &huart3) {
        const uint8_t byte = tof_right.rx_byte;
        tofsense_ensure_rx_armed(&huart3, &tof_right, &tof_right_rx);
        tofsense_rx_push(&tof_right_rx, byte);
    }
#else
    (void)uart;
#endif
}

void sensor_hub_on_uart_error(UART_HandleTypeDef *uart)
{
#if TOF_PROTOCOL_MODE == TOF_PROTOCOL_TOFSENSE_M
    if (uart == &huart2) {
        ++tof_left_rx.uart_errors;
        tofsense_ensure_rx_armed(&huart2, &tof_left, &tof_left_rx);
    } else if (uart == &huart3) {
        ++tof_right_rx.uart_errors;
        tofsense_ensure_rx_armed(&huart3, &tof_right, &tof_right_rx);
    }
#else
    (void)uart;
#endif
}

void sensor_hub_on_tim_ic_capture(TIM_HandleTypeDef *timer)
{
#if ULTRASONIC_PROTOCOL_MODE == ULTRASONIC_MODE_TRIGGER_ECHO
    uint16_t capture;

    if (timer != &htim4 || timer->Channel != us_active_channels[us_index]) {
        return;
    }
    if (ultrasonic_measurement_pending == 0U
        || ultrasonic_pulse_ready != 0U) {
        return;
    }
    capture = (uint16_t)HAL_TIM_ReadCapturedValue(timer, us_channels[us_index]);
    if (ultrasonic_wait_falling == 0U) {
        ultrasonic_rising_us = capture;
        ultrasonic_rising_ms = HAL_GetTick();
        ultrasonic_wait_falling = 1U;
        __HAL_TIM_SET_CAPTUREPOLARITY(timer,
                                      us_channels[us_index],
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
        return;
    }

    ultrasonic_captured_pulse_us = (uint16_t)(capture - ultrasonic_rising_us);
    ultrasonic_long_pulse = (HAL_GetTick() - ultrasonic_rising_ms >= 60U) ? 1U : 0U;
    ultrasonic_wait_falling = 0U;
    ultrasonic_pulse_ready = 1U;
    __HAL_TIM_SET_CAPTUREPOLARITY(timer,
                                  us_channels[us_index],
                                  TIM_INPUTCHANNELPOLARITY_RISING);
#else
    (void)timer;
#endif
}
