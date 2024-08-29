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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NE_ENC_A_Pin GPIO_PIN_0
#define NE_ENC_A_GPIO_Port GPIOA
#define NE_ENC_B_Pin GPIO_PIN_1
#define NE_ENC_B_GPIO_Port GPIOA
#define NW_ENC_A_Pin GPIO_PIN_5
#define NW_ENC_A_GPIO_Port GPIOA
#define SW_ENC_A_Pin GPIO_PIN_6
#define SW_ENC_A_GPIO_Port GPIOA
#define SW_ENC_B_Pin GPIO_PIN_7
#define SW_ENC_B_GPIO_Port GPIOA
#define NW_IN1_Pin GPIO_PIN_0
#define NW_IN1_GPIO_Port GPIOB
#define NW_IN2_Pin GPIO_PIN_1
#define NW_IN2_GPIO_Port GPIOB
#define SW_IN1_Pin GPIO_PIN_2
#define SW_IN1_GPIO_Port GPIOB
#define SW_IN2_Pin GPIO_PIN_7
#define SW_IN2_GPIO_Port GPIOE
#define SE_IN1_Pin GPIO_PIN_8
#define SE_IN1_GPIO_Port GPIOE
#define NW_PWM_Pin GPIO_PIN_9
#define NW_PWM_GPIO_Port GPIOE
#define SE_IN2_Pin GPIO_PIN_10
#define SE_IN2_GPIO_Port GPIOE
#define SW_PWM_Pin GPIO_PIN_11
#define SW_PWM_GPIO_Port GPIOE
#define NE_IN1_Pin GPIO_PIN_12
#define NE_IN1_GPIO_Port GPIOE
#define SE_PWM_Pin GPIO_PIN_13
#define SE_PWM_GPIO_Port GPIOE
#define NE_PWM_Pin GPIO_PIN_14
#define NE_PWM_GPIO_Port GPIOE
#define NE_IN2_Pin GPIO_PIN_15
#define NE_IN2_GPIO_Port GPIOE
#define SE_ENC_A_Pin GPIO_PIN_12
#define SE_ENC_A_GPIO_Port GPIOD
#define SE_ENC_B_Pin GPIO_PIN_13
#define SE_ENC_B_GPIO_Port GPIOD
#define NW_SERVO_Pin GPIO_PIN_6
#define NW_SERVO_GPIO_Port GPIOC
#define SW_SERVO_Pin GPIO_PIN_7
#define SW_SERVO_GPIO_Port GPIOC
#define SE_SERVO_Pin GPIO_PIN_8
#define SE_SERVO_GPIO_Port GPIOC
#define NE_SERVO_Pin GPIO_PIN_9
#define NE_SERVO_GPIO_Port GPIOC
#define NW_ENC_B_Pin GPIO_PIN_3
#define NW_ENC_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
