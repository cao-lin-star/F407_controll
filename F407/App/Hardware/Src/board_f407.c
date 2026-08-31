#include "board.h"

#include <math.h>

#include "board_config.h"
#include "main.h"
#include "tim.h"

static uint16_t left_encoder_previous;
static uint16_t right_encoder_previous;

void board_system_clock_config(void)
{
    /* Generated SystemClock_Config() in Core/Src/main.c owns F407 clocks. */
}

static void check_hal(HAL_StatusTypeDef status)
{
    if (status != HAL_OK) {
        board_fatal_error();
    }
}

void board_peripherals_init(void)
{
    check_hal(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1));
    check_hal(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2));
    check_hal(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3));
    check_hal(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4));
    check_hal(HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL));
    check_hal(HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL));

    left_encoder_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    right_encoder_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    board_motor_stop();
}

static void motor_set_one(float percent,
                          int command_sign,
                          uint32_t rpwm_channel,
                          uint32_t lpwm_channel,
                          GPIO_TypeDef *enable_port,
                          uint16_t enable_pin)
{
    uint32_t compare;

    percent *= (float)command_sign;
    if (percent > MOTOR_OUTPUT_LIMIT_PERCENT) {
        percent = MOTOR_OUTPUT_LIMIT_PERCENT;
    } else if (percent < -MOTOR_OUTPUT_LIMIT_PERCENT) {
        percent = -MOTOR_OUTPUT_LIMIT_PERCENT;
    }

    if (fabsf(percent) < 0.001f) {
        __HAL_TIM_SET_COMPARE(&htim1, rpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, lpwm_channel, 0U);
#if MOTOR_STOP_BRAKE
        HAL_GPIO_WritePin(enable_port, enable_pin, GPIO_PIN_SET);
#else
        HAL_GPIO_WritePin(enable_port, enable_pin, GPIO_PIN_RESET);
#endif
        return;
    }

    compare = (uint32_t)(fabsf(percent)
                         * (float)MOTOR_PWM_PERIOD_COUNTS / 100.0f);
    HAL_GPIO_WritePin(enable_port, enable_pin, GPIO_PIN_SET);
    if (percent > 0.0f) {
        __HAL_TIM_SET_COMPARE(&htim1, lpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, rpwm_channel, compare);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, rpwm_channel, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, lpwm_channel, compare);
    }
}

void board_motor_set_percent(float left_percent, float right_percent)
{
    motor_set_one(left_percent,
                  LEFT_MOTOR_COMMAND_SIGN,
                  TIM_CHANNEL_1,
                  TIM_CHANNEL_2,
                  MOTOR_LEFT_EN_GPIO_Port,
                  MOTOR_LEFT_EN_Pin);
    motor_set_one(right_percent,
                  RIGHT_MOTOR_COMMAND_SIGN,
                  TIM_CHANNEL_3,
                  TIM_CHANNEL_4,
                  MOTOR_RIGHT_EN_GPIO_Port,
                  MOTOR_RIGHT_EN_Pin);
}

void board_motor_stop(void)
{
    motor_set_one(0.0f, 1, TIM_CHANNEL_1, TIM_CHANNEL_2,
                  MOTOR_LEFT_EN_GPIO_Port, MOTOR_LEFT_EN_Pin);
    motor_set_one(0.0f, 1, TIM_CHANNEL_3, TIM_CHANNEL_4,
                  MOTOR_RIGHT_EN_GPIO_Port, MOTOR_RIGHT_EN_Pin);
}

void board_motor_force_stop_from_fault(void)
{
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = 0U;
    GPIOE->BSRR = ((uint32_t)(MOTOR_LEFT_EN_Pin | MOTOR_RIGHT_EN_Pin) << 16U);
}

void board_encoder_sample(int32_t *left_delta, int32_t *right_delta)
{
    const uint16_t left_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    const uint16_t right_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /* Return raw timer direction; control_update() applies each sign once. */
    *left_delta = (int32_t)((int16_t)(left_now - left_encoder_previous));
    *right_delta = (int32_t)((int16_t)(right_now - right_encoder_previous));
    left_encoder_previous = left_now;
    right_encoder_previous = right_now;
}

void board_led_set(uint8_t on)
{
    /* Wildfire F407 LEDs are active-low. */
    HAL_GPIO_WritePin(STATUS_LED_R_GPIO_Port,
                      STATUS_LED_R_Pin,
                      on != 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool board_estop_active(void)
{
    return HAL_GPIO_ReadPin(ESTOP_N_GPIO_Port, ESTOP_N_Pin) == GPIO_PIN_RESET;
}

void board_fatal_error(void)
{
    __disable_irq();
    board_motor_force_stop_from_fault();
    GPIOF->BSRR = ((uint32_t)STATUS_LED_R_Pin << 16U);
    for (;;) {
        __NOP();
    }
}
