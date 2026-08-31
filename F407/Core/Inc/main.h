/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STATUS_LED_R_Pin GPIO_PIN_6
#define STATUS_LED_R_GPIO_Port GPIOF
#define PS2_DATA_Pin GPIO_PIN_4
#define PS2_DATA_GPIO_Port GPIOA
#define PS2_CMD_Pin GPIO_PIN_5
#define PS2_CMD_GPIO_Port GPIOA
#define PS2_ATT_Pin GPIO_PIN_6
#define PS2_ATT_GPIO_Port GPIOA
#define PS2_CLK_Pin GPIO_PIN_7
#define PS2_CLK_GPIO_Port GPIOA
#define ULTRASONIC_TRIG_Pin GPIO_PIN_0
#define ULTRASONIC_TRIG_GPIO_Port GPIOC
#define MOTOR_LEFT_EN_Pin GPIO_PIN_8
#define MOTOR_LEFT_EN_GPIO_Port GPIOE
#define MOTOR_LEFT_RPWM_Pin GPIO_PIN_9
#define MOTOR_LEFT_RPWM_GPIO_Port GPIOE
#define MOTOR_RIGHT_EN_Pin GPIO_PIN_10
#define MOTOR_RIGHT_EN_GPIO_Port GPIOE
#define MOTOR_LEFT_LPWM_Pin GPIO_PIN_11
#define MOTOR_LEFT_LPWM_GPIO_Port GPIOE
#define MOTOR_RIGHT_RPWM_Pin GPIO_PIN_13
#define MOTOR_RIGHT_RPWM_GPIO_Port GPIOE
#define MOTOR_RIGHT_LPWM_Pin GPIO_PIN_14
#define MOTOR_RIGHT_LPWM_GPIO_Port GPIOE
#define TOF_RIGHT_TX_Pin GPIO_PIN_8
#define TOF_RIGHT_TX_GPIO_Port GPIOD
#define TOF_RIGHT_RX_Pin GPIO_PIN_9
#define TOF_RIGHT_RX_GPIO_Port GPIOD
#define IR_LEFT_N_Pin GPIO_PIN_2
#define IR_LEFT_N_GPIO_Port GPIOG
#define IR_RIGHT_N_Pin GPIO_PIN_3
#define IR_RIGHT_N_GPIO_Port GPIOG
#define ESTOP_N_Pin GPIO_PIN_4
#define ESTOP_N_GPIO_Port GPIOG
#define RK_UART_TX_Pin GPIO_PIN_9
#define RK_UART_TX_GPIO_Port GPIOA
#define RK_UART_RX_Pin GPIO_PIN_10
#define RK_UART_RX_GPIO_Port GPIOA
#define ENC_LEFT_A_Pin GPIO_PIN_15
#define ENC_LEFT_A_GPIO_Port GPIOA
#define DEBUG_UART_TX_Pin GPIO_PIN_10
#define DEBUG_UART_TX_GPIO_Port GPIOC
#define DEBUG_UART_RX_Pin GPIO_PIN_11
#define DEBUG_UART_RX_GPIO_Port GPIOC
#define TOF_LEFT_TX_Pin GPIO_PIN_5
#define TOF_LEFT_TX_GPIO_Port GPIOD
#define TOF_LEFT_RX_Pin GPIO_PIN_6
#define TOF_LEFT_RX_GPIO_Port GPIOD
#define ENC_LEFT_B_Pin GPIO_PIN_3
#define ENC_LEFT_B_GPIO_Port GPIOB
#define ENC_RIGHT_A_Pin GPIO_PIN_4
#define ENC_RIGHT_A_GPIO_Port GPIOB
#define ENC_RIGHT_B_Pin GPIO_PIN_5
#define ENC_RIGHT_B_GPIO_Port GPIOB
#define ULTRASONIC_ECHO_Pin GPIO_PIN_6
#define ULTRASONIC_ECHO_GPIO_Port GPIOB
#define MPU6050_SCL_Pin GPIO_PIN_8
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_9
#define MPU6050_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
