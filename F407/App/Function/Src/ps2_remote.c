#include "ps2_remote.h"

#include <string.h>

#include "board_config.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define PS2_FRAME_BYTES           9U
#define PS2_ID_DIGITAL         0x41U
#define PS2_ID_ANALOG_GREEN    0x53U
#define PS2_ID_ANALOG_RED      0x73U
#define PS2_ID_ANALOG_PRESSURE 0x79U
#define PS2_READY_BYTE         0x5AU

#define PS2_BTN1_SELECT        (1U << 0)
#define PS2_BTN1_START         (1U << 3)
#define PS2_BTN2_R1            (1U << 3)

static const uint8_t request_frame[PS2_FRAME_BYTES] = {
    0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};

static ps2_remote_status_t status;
static uint32_t next_poll_ms;
static uint8_t previous_btn1_raw = 0xFFU;
static uint8_t start_release_seen;

static void delay_us(uint32_t microseconds)
{
    const uint32_t cycles = (SystemCoreClock / 1000000U) * microseconds;
    const uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    }
}

static uint8_t transfer_byte(uint8_t output)
{
    uint8_t input = 0U;
    uint8_t mask;

    for (mask = 0x01U; mask != 0U; mask <<= 1U) {
        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PS2_CMD_GPIO_Port,
                          PS2_CMD_Pin,
                          (output & mask) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
        delay_us(PS2_CLOCK_HALF_PERIOD_US);
        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
        if (HAL_GPIO_ReadPin(PS2_DATA_GPIO_Port, PS2_DATA_Pin) == GPIO_PIN_SET) {
            input |= mask;
        }
        delay_us(PS2_CLOCK_HALF_PERIOD_US);
    }
    HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
    return input;
}

static void read_frame(uint8_t *response)
{
    uint8_t i;

    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_RESET);
    delay_us(PS2_CLOCK_HALF_PERIOD_US);
    for (i = 0U; i < PS2_FRAME_BYTES; ++i) {
        response[i] = transfer_byte(request_frame[i]);
        delay_us(PS2_CLOCK_HALF_PERIOD_US);
    }
    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_SET);
}

static uint8_t id_supported(uint8_t id)
{
    return (uint8_t)(id == PS2_ID_DIGITAL
                     || id == PS2_ID_ANALOG_GREEN
                     || id == PS2_ID_ANALOG_RED
                     || id == PS2_ID_ANALOG_PRESSURE);
}

static uint8_t analog_motion_available(uint8_t id)
{
    return (uint8_t)(id == PS2_ID_ANALOG_RED
                     || id == PS2_ID_ANALOG_PRESSURE);
}

static uint8_t button_pressed(uint8_t raw, uint8_t mask)
{
    return (uint8_t)((raw & mask) == 0U);
}

static uint8_t sticks_centered(const uint8_t *frame)
{
    const int16_t rx = (int16_t)frame[5] - 128;
    const int16_t ry = (int16_t)frame[6] - 128;
    const int16_t lx = (int16_t)frame[7] - 128;
    const int16_t ly = (int16_t)frame[8] - 128;

    return (uint8_t)(rx >= -(int16_t)PS2_STICK_DEADBAND_RAW
                     && rx <= (int16_t)PS2_STICK_DEADBAND_RAW
                     && ry >= -(int16_t)PS2_STICK_DEADBAND_RAW
                     && ry <= (int16_t)PS2_STICK_DEADBAND_RAW
                     && lx >= -(int16_t)PS2_STICK_DEADBAND_RAW
                     && lx <= (int16_t)PS2_STICK_DEADBAND_RAW
                     && ly >= -(int16_t)PS2_STICK_DEADBAND_RAW
                     && ly <= (int16_t)PS2_STICK_DEADBAND_RAW);
}

static float axis_value(uint8_t raw)
{
    const int16_t delta = (int16_t)raw - 128;
    const int16_t magnitude = delta < 0 ? (int16_t)-delta : delta;
    const int16_t usable = 128 - (int16_t)PS2_STICK_DEADBAND_RAW;
    float result;

    if (magnitude <= (int16_t)PS2_STICK_DEADBAND_RAW) {
        return 0.0f;
    }
    result = (float)(magnitude - (int16_t)PS2_STICK_DEADBAND_RAW)
           / (float)usable;
    if (result > 1.0f) {
        result = 1.0f;
    }
    return delta < 0 ? -result : result;
}

void ps2_remote_init(uint32_t now_ms)
{
    memset(&status, 0, sizeof(status));
    status.btn1_raw = 0xFFU;
    status.btn2_raw = 0xFFU;
    status.right_x = 0x80U;
    status.right_y = 0x80U;
    status.left_x = 0x80U;
    status.left_y = 0x80U;
    previous_btn1_raw = 0xFFU;
    start_release_seen = 0U;
    next_poll_ms = now_ms;

#if PS2_REMOTE_ENABLE
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_ATT_GPIO_Port, PS2_ATT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
#endif
}

ps2_remote_action_t ps2_remote_process(uint32_t now_ms,
                                       float *linear_mps,
                                       float *angular_rps)
{
#if PS2_REMOTE_ENABLE
    uint8_t frame[PS2_FRAME_BYTES];
    uint8_t start_pressed;
    uint8_t start_was_pressed;
    uint8_t select_pressed;
    uint8_t valid;

    if (status.connected != 0U
        && (uint32_t)(now_ms - status.last_valid_ms) > PS2_LINK_TIMEOUT_MS) {
        status.connected = 0U;
        status.manual_mode = 0U;
        status.deadman_held = 0U;
        status.stop_latched = 0U;
        start_release_seen = 0U;
        previous_btn1_raw = 0xFFU;
        status.linear_mps = 0.0f;
        status.angular_rps = 0.0f;
        return PS2_REMOTE_ACTION_RELEASE;
    }

    if ((int32_t)(now_ms - next_poll_ms) < 0) {
        return PS2_REMOTE_ACTION_NONE;
    }
    next_poll_ms += PS2_POLL_PERIOD_MS;
    if ((int32_t)(now_ms - next_poll_ms) >= 0) {
        next_poll_ms = now_ms + PS2_POLL_PERIOD_MS;
    }

    read_frame(frame);
    valid = (uint8_t)(id_supported(frame[1]) != 0U
                      && frame[2] == PS2_READY_BYTE);
    if (valid == 0U) {
        ++status.invalid_frames;
        if (status.manual_mode != 0U
            && (status.last_valid_ms == 0U
                || (uint32_t)(now_ms - status.last_valid_ms)
                   > PS2_LINK_TIMEOUT_MS)) {
            status.connected = 0U;
            status.manual_mode = 0U;
            status.deadman_held = 0U;
            status.stop_latched = 0U;
            start_release_seen = 0U;
            previous_btn1_raw = 0xFFU;
            status.linear_mps = 0.0f;
            status.angular_rps = 0.0f;
            return PS2_REMOTE_ACTION_RELEASE;
        }
        return PS2_REMOTE_ACTION_NONE;
    }

    ++status.valid_frames;
    status.last_valid_ms = now_ms;
    status.connected = 1U;
    status.mode_id = frame[1];
    status.btn1_raw = frame[3];
    status.btn2_raw = frame[4];
    status.right_x = frame[5];
    status.right_y = frame[6];
    status.left_x = frame[7];
    status.left_y = frame[8];
    status.deadman_held = button_pressed(frame[4], PS2_BTN2_R1);

    start_pressed = button_pressed(frame[3], PS2_BTN1_START);
    start_was_pressed = button_pressed(previous_btn1_raw, PS2_BTN1_START);
    select_pressed = button_pressed(frame[3], PS2_BTN1_SELECT);
    previous_btn1_raw = frame[3];
    if (start_pressed == 0U) {
        start_release_seen = 1U;
    }

    if (status.manual_mode != 0U
        && analog_motion_available(status.mode_id) == 0U) {
        status.manual_mode = 0U;
        status.stop_latched = 1U;
        status.linear_mps = 0.0f;
        status.angular_rps = 0.0f;
        return PS2_REMOTE_ACTION_STOP;
    }
    if (select_pressed != 0U) {
        status.manual_mode = 0U;
        status.stop_latched = 1U;
        status.linear_mps = 0.0f;
        status.angular_rps = 0.0f;
        return PS2_REMOTE_ACTION_STOP;
    }

    if (start_pressed != 0U && start_was_pressed == 0U
        && start_release_seen != 0U
        && sticks_centered(frame) != 0U
        && status.deadman_held == 0U
        && analog_motion_available(status.mode_id) != 0U) {
        start_release_seen = 0U;
        if (status.stop_latched != 0U) {
            status.stop_latched = 0U;
        } else {
            status.manual_mode ^= 1U;
        }
        status.linear_mps = 0.0f;
        status.angular_rps = 0.0f;
        return PS2_REMOTE_ACTION_STOP;
    }

    if (status.manual_mode == 0U) {
        status.linear_mps = 0.0f;
        status.angular_rps = 0.0f;
        return PS2_REMOTE_ACTION_NONE;
    }
    if (status.stop_latched != 0U) {
        status.manual_mode = 0U;
        status.linear_mps = 0.0f;
        status.angular_rps = 0.0f;
        return PS2_REMOTE_ACTION_STOP;
    }
    if (status.deadman_held == 0U) {
        status.linear_mps = 0.0f;
        status.angular_rps = 0.0f;
        return PS2_REMOTE_ACTION_RELEASE;
    }

    status.linear_mps = -axis_value(status.left_y)
                      * PS2_MANUAL_MAX_LINEAR_MPS;
    status.angular_rps = -axis_value(status.right_x)
                       * PS2_MANUAL_MAX_ANGULAR_RPS;
    if (linear_mps != NULL) {
        *linear_mps = status.linear_mps;
    }
    if (angular_rps != NULL) {
        *angular_rps = status.angular_rps;
    }
    return PS2_REMOTE_ACTION_VELOCITY;
#else
    (void)now_ms;
    (void)linear_mps;
    (void)angular_rps;
    return PS2_REMOTE_ACTION_NONE;
#endif
}

bool ps2_remote_manual_active(void)
{
    return status.connected != 0U && status.manual_mode != 0U;
}

void ps2_remote_force_stop(void)
{
    if (status.manual_mode != 0U) {
        status.manual_mode = 0U;
        status.stop_latched = 1U;
        status.deadman_held = 0U;
        status.linear_mps = 0.0f;
        status.angular_rps = 0.0f;
    }
}

void ps2_remote_get_status(ps2_remote_status_t *out)
{
    if (out != NULL) {
        *out = status;
    }
}
