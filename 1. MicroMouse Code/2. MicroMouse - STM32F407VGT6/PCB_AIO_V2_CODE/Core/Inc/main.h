/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
// static void MX_TIM3_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIT_SW_0_Pin GPIO_PIN_2
#define BIT_SW_0_GPIO_Port GPIOE
#define BIT_SW_1_Pin GPIO_PIN_3
#define BIT_SW_1_GPIO_Port GPIOE
#define BIT_SW_2_Pin GPIO_PIN_4
#define BIT_SW_2_GPIO_Port GPIOE
#define BIT_SW_3_Pin GPIO_PIN_5
#define BIT_SW_3_GPIO_Port GPIOE
#define BIT_SW_4_Pin GPIO_PIN_6
#define BIT_SW_4_GPIO_Port GPIOE
#define IR_SENSOR_FR_Pin GPIO_PIN_0
#define IR_SENSOR_FR_GPIO_Port GPIOC
#define IR_SENSOR_RR_Pin GPIO_PIN_1
#define IR_SENSOR_RR_GPIO_Port GPIOC
#define RIGHT_ENCODER_A_Pin GPIO_PIN_0
#define RIGHT_ENCODER_A_GPIO_Port GPIOA
#define RIGHT_ENCODER_B_Pin GPIO_PIN_1
#define RIGHT_ENCODER_B_GPIO_Port GPIOA
#define IR_SENSOR_BR_Pin GPIO_PIN_2
#define IR_SENSOR_BR_GPIO_Port GPIOA
#define IR_SENSOR_BL_Pin GPIO_PIN_3
#define IR_SENSOR_BL_GPIO_Port GPIOA
#define IR_SENSOR_RL_Pin GPIO_PIN_6
#define IR_SENSOR_RL_GPIO_Port GPIOA
#define IR_SENSOR_FL_Pin GPIO_PIN_4
#define IR_SENSOR_FL_GPIO_Port GPIOC
#define IR_SENSOR_FF_Pin GPIO_PIN_1
#define IR_SENSOR_FF_GPIO_Port GPIOB
#define LEFT_MOTOR_IN1_Pin GPIO_PIN_7
#define LEFT_MOTOR_IN1_GPIO_Port GPIOE
#define LEFT_MOTOR_IN2_Pin GPIO_PIN_8
#define LEFT_MOTOR_IN2_GPIO_Port GPIOE
#define LEFT_MOTOR_PWM_Pin GPIO_PIN_9
#define LEFT_MOTOR_PWM_GPIO_Port GPIOE
#define RIGHT_MOTOR_PWM_Pin GPIO_PIN_11
#define RIGHT_MOTOR_PWM_GPIO_Port GPIOE
#define RIGHT_MOTOR_IN1_Pin GPIO_PIN_12
#define RIGHT_MOTOR_IN1_GPIO_Port GPIOE
#define RIGHT_MOTOR_IN2_Pin GPIO_PIN_13
#define RIGHT_MOTOR_IN2_GPIO_Port GPIOE
#define LEFT_ENCODER_A_Pin GPIO_PIN_15
#define LEFT_ENCODER_A_GPIO_Port GPIOA
#define LED_0_Pin GPIO_PIN_0
#define LED_0_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOD
#define LEFT_ENCODER_B_Pin GPIO_PIN_3
#define LEFT_ENCODER_B_GPIO_Port GPIOB
#define BUTTON_0_Pin GPIO_PIN_0
#define BUTTON_0_GPIO_Port GPIOE
#define BUTTON_1_Pin GPIO_PIN_1
#define BUTTON_1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
