/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "stdio.h"

#include "AML_MPUSensor.h"
#include "AML_LaserSensor.h"
#include "AML_Keyboard.h"
#include "AML_Encoder.h"
#include "AML_MotorControl.h"
#include "AML_DebugDevice.h"
#include "AML_Remote.h"
#include "AML_Parameter.h"

// #include "flood.h"
#include "solver.h"
// #include "AML_Remote.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t ButtonPressed;

int16_t debug[100];
uint8_t checkWorking[] = {0xFF, 0xAA, 0x52};

// 1 to turn on, 0 to turn off

volatile uint8_t ReadButton;
volatile uint8_t RemarkAfterTurnMode = 0;
volatile uint8_t RemarkWallMode = 1;
volatile uint8_t RemarkAfterBackwardMode = 1;

double testAngle;
// int16_t LeftValue, RightValue;
double LeftSpeed, RightSpeed;

uint32_t delay = 500;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // choose which algorithm to use in the beginning of the run
// int algorithm;
// struct dist_maze distances;
// struct wall_maze cell_walls_info;
// struct stack update_stack;
// struct stack move_queue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void TestLaser(void);
void TestMotor(void);
void TestMPU(void);
void TestTurning(void);
void EncoderTest(void);
void TestMPUFollow(void);
void SystemTest(void);

void TurnOnRemarkAfterTurn(void);
void TurnOffRemarkAfterTurn(void);

void TurnOnRemarkWall(void);
void TurnOffRemarkWall(void);

void SetTicks(void);

void TestMode(void);
void RemarkMode(void);

void Run(int);
void ShortestPath();
void RunMode();

void RunNewAlgorithm();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// int fputc(int ch, FILE *f)
// {
//   ITM_SendChar(ch); // send method for SWV
//   return (ch);
// }

////////////////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  AML_MPUSensor_Setup();
  // AML_Keyboard_Setup();
  AML_LaserSensor_Setup();
  AML_Encoder_Setup();
  AML_MotorControl_Setup();
  AML_Remote_Setup();

  HAL_TIM_Base_Start_IT(&htim10);

  ReadButton = 8;

  // for (int i = 0; i < 3; i++)
  // {
  //   AML_LaserSensor_ReadAll();
  //   HAL_Delay(35);
  // }

  AML_LaserSensor_ReadAll();
  AML_LaserSensor_ReadAll();
  AML_LaserSensor_ReadAll();

  AML_DebugDevice_BuzzerBeep(20);


  // AML_DebugDevice_TurnOnIT();

  // HAL_Delay(5000);

  // AML_DebugDevice_TurnOffIT();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (ButtonPressed)
    {
      AML_DebugDevice_BuzzerBeep(20);
      ButtonPressed = 0;
    }

    // AML_LaserSensor_ReadAll();
    // for (int i = 0; i < 5; i++)
    // {
    //   debug[i] = AML_LaserSensor_ReadSingleWithFillter(i);
    // }

    // AML_LaserSensor_ReadAll();

    // AML_LaserSensor_ReadSingleWithFillter(FF);

    if (ReadButton == 0) // set left wall value
    {
      SetTicks();
      ReadButton = 8;
    }
    else if (ReadButton == 1) // set right wall value
    {
      RemarkMode();
      ReadButton = 8;
    }
    else if (ReadButton == 2)
    {
      TestMode();
      ReadButton = 8;
    }
    else if (ReadButton == 3)
    {
      // ShortestPath();
      ReadButton = 8;
    }
    else if (ReadButton == 4)
    {
      // RunMode();
      ReadButton = 8;
      RunNewAlgorithm();
    }

    testAngle = AML_MPUSensor_GetAngle();

    // AML_MotorControl_LeftPWM(-6);
    // AML_MotorControl_RightPWM(6);

    // debug[13] = AML_Encoder_GetLeftValue();
    // debug[14] = AML_Encoder_GetRightValue();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8399;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 250;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8399;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 200;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_10 | STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | XSHUT_FF_Pin | BIN2_Pin | XSHUT_BR_Pin | BIN1_Pin | XSHUT_BL_Pin | AIN2_Pin | XSHUT_FR_Pin | AIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XSHUT_FL_GPIO_Port, XSHUT_FL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC8 PC10
                           STBY_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_10 | STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 XSHUT_FF_Pin BIN2_Pin XSHUT_BR_Pin
                           BIN1_Pin XSHUT_BL_Pin AIN2_Pin XSHUT_FR_Pin
                           AIN1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | XSHUT_FF_Pin | BIN2_Pin | XSHUT_BR_Pin | BIN1_Pin | XSHUT_BL_Pin | AIN2_Pin | XSHUT_FR_Pin | AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : XSHUT_FL_Pin */
  GPIO_InitStruct.Pin = XSHUT_FL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XSHUT_FL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TestLaser(void)
{
  AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);

  AML_LaserSensor_ReadAll();

  AML_DebugDevice_BuzzerBeep(100);
  HAL_Delay(200);

  for (int i = 0; i < 5; i++)
  {
    if (AML_LaserSensor_ReadSingleWithFillter(i) != 0)
    {
      AML_DebugDevice_TurnOnLED(i);
    }
  }

  HAL_Delay(1000); // �?ợi 1 giây

  AML_DebugDevice_BuzzerBeep(100);
}

void TestMotor(void)
{
  AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);

  AML_Encoder_ResetLeftValue();
  AML_Encoder_ResetRightValue();

  while (AML_Encoder_GetLeftValue() < 1000 | AML_Encoder_GetRightValue() < 1000)
  {
    AML_MotorControl_LeftPWM(20);
    AML_MotorControl_RightPWM(20);
  }

  AML_MotorControl_Stop();

  AML_DebugDevice_TurnOnLED(0);

  AML_DebugDevice_BuzzerBeep(50);
  HAL_Delay(50);
  AML_DebugDevice_BuzzerBeep(50);
  HAL_Delay(50);

  AML_Encoder_ResetLeftValue();
  AML_Encoder_ResetRightValue();

  while (AML_Encoder_GetLeftValue() > -1000 | AML_Encoder_GetRightValue() > -1000)
  {
    AML_MotorControl_LeftPWM(-20);
    AML_MotorControl_RightPWM(-20);
  }

  AML_MotorControl_Stop();

  AML_DebugDevice_TurnOnLED(1);

  AML_DebugDevice_BuzzerBeep(50);
  HAL_Delay(50);
  AML_DebugDevice_BuzzerBeep(50);
  HAL_Delay(50);

  HAL_Delay(1000);

  AML_Encoder_ResetLeftValue();
  AML_Encoder_ResetRightValue();
}

void TestMPU()
{
  AML_DebugDevice_SetAllLED(GPIO_PIN_SET);

  while (AML_MPUSensor_GetAngle() == 0)
  {
    AML_DebugDevice_SetAllLED(GPIO_PIN_SET);

    HAL_Delay(100);

    AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);

    HAL_Delay(100);
  }

  AML_DebugDevice_BuzzerBeep(50);
  HAL_Delay(50);
  AML_DebugDevice_BuzzerBeep(50);
  HAL_Delay(50);
  AML_DebugDevice_BuzzerBeep(50);
  HAL_Delay(50);

  AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
}

void TestTurning()
{
  RemarkAfterTurnMode = 1;

  AML_MPUSensor_ResetAngle();

  AML_MotorControl_TurnLeft90();
  AML_MotorControl_Stop();

  HAL_Delay(1000);

  AML_MotorControl_TurnRight90();
  AML_MotorControl_Stop();

  HAL_Delay(1000);

  AML_MotorControl_TurnLeft180();
  AML_MotorControl_Stop();

  HAL_Delay(1000);

  AML_MotorControl_TurnRight180();
  AML_MotorControl_Stop();

  RemarkAfterTurnMode = 0;
}

void EncoderTest()
{
  AML_MotorControl_TurnOnWallFollow();

  while (AML_Encoder_GetLeftValue() < ENCODER_TICKS_ONE_CELL && ReadButton != 2)
  {
  }

  AML_MotorControl_TurnOffWallFollow();
  AML_MotorControl_ShortBreak('F');
  AML_Encoder_ResetLeftValue();
  AML_DebugDevice_BuzzerBeep(30);

  HAL_Delay(1500);
}

void TestMPUFollow()
{
  AML_MPUSensor_ResetAngle();

  while (1)
  {
    AML_MotorControl_MPUFollow(0);
  }
}

void SystemTest(void)
{
  TestLaser();
  TestMotor();
  TestMPU();
}

/////////////////////////////////////////////////////////////////////////////////////

void TurnOnRemarkAfterTurn()
{
  RemarkAfterTurnMode = 1;
}

void TurnOffRemarkAfterTurn()
{
  RemarkAfterTurnMode = 0;
}

void TurnOnRemarkWall()
{
  RemarkWallMode = 1;
}

void TurnOffRemarkWall()
{
  RemarkWallMode = 0;
}

/////////////////////////////////////////////////////////////////////////////////////

void SetTicks()
{
  HAL_Delay(300);

  ReadButton = 8;

  AML_DebugDevice_SetAllLED(GPIO_PIN_SET);

  while (ReadButton == 8)
  {
  }

  AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);

  if (ReadButton == 0)
  {
    AML_MotorControl_SetCenterPosition();
    AML_DebugDevice_BuzzerBeep(20);
    AML_DebugDevice_TurnOnLED(ReadButton);
    HAL_Delay(300);
    AML_DebugDevice_TurnOffLED(ReadButton);
  }
  else if (ReadButton == 1)
  {
    // SetBeforeTurnTicks(0);
    AML_DebugDevice_BuzzerBeep(20);
    // AML_DebugDevice_TurnOnLED(ReadButton);
    // HAL_Delay(300);
    // AML_DebugDevice_TurnOffLED(ReadButton);
  }
  else if (ReadButton == 2)
  {
    // SetBeforeTurnTicks(1);
    AML_DebugDevice_BuzzerBeep(20);
    // AML_DebugDevice_TurnOnLED(ReadButton);
    // HAL_Delay(300);
    // AML_DebugDevice_TurnOffLED(ReadButton);
  }
  else if (ReadButton == 3)
  {
    // SetAfterTurnTicks(0);
    AML_DebugDevice_BuzzerBeep(20);
    // AML_DebugDevice_TurnOnLED(ReadButton);
    // HAL_Delay(300);
    // AML_DebugDevice_TurnOffLED(ReadButton);
  }
  else if (ReadButton == 4)
  {
    // SetAfterTurnTicks(1);
    AML_DebugDevice_BuzzerBeep(20);
    // AML_DebugDevice_TurnOnLED(ReadButton);
    // HAL_Delay(300);
    // AML_DebugDevice_TurnOffLED(ReadButton);
  }
}

void TestMode()
{

  AML_DebugDevice_SetAllLED(GPIO_PIN_SET);

  HAL_Delay(300);
  ReadButton = 8;

  while (ReadButton == 8)
  {
  }

  AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);

  if (ReadButton == 0)
  {
    SystemTest();
  }
  else if (ReadButton == 1)
  {
    TestTurning();
  }
  else if (ReadButton == 2)
  {
    TestMPUFollow();
  }
  else if (ReadButton == 3)
  {
    // advanceOneCellVisited();
    // AML_MotorControl_ShortBreak('F');
  }
}

void RemarkMode()
{
  HAL_Delay(300);

  ReadButton = 8;

  AML_DebugDevice_SetAllLED(GPIO_PIN_SET);

  while (ReadButton == 8)
  {
  }

  AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);

  if (ReadButton == 0)
  {
    TurnOffRemarkAfterTurn();
    AML_DebugDevice_BuzzerBeep(20);
    AML_DebugDevice_TurnOnLED(ReadButton);
    HAL_Delay(300);
    AML_DebugDevice_TurnOffLED(ReadButton);
  }
  else if (ReadButton == 1)
  {
    TurnOnRemarkAfterTurn();
    AML_DebugDevice_BuzzerBeep(20);
    AML_DebugDevice_TurnOnLED(ReadButton);
    HAL_Delay(300);
    AML_DebugDevice_TurnOffLED(ReadButton);
  }
  else if (ReadButton == 2)
  {
    TurnOffRemarkWall();
    AML_DebugDevice_BuzzerBeep(20);
    AML_DebugDevice_TurnOnLED(ReadButton);
    HAL_Delay(300);
    AML_DebugDevice_TurnOffLED(ReadButton);
  }
  else if (ReadButton == 3)
  {
    TurnOnRemarkAfterTurn();
    AML_DebugDevice_BuzzerBeep(20);
    AML_DebugDevice_TurnOnLED(ReadButton);
    HAL_Delay(300);
    AML_DebugDevice_TurnOffLED(ReadButton);
  }
  else if (ReadButton == 4)
  {
    RemarkAfterBackwardMode = 0;
    AML_DebugDevice_BuzzerBeep(20);
    AML_DebugDevice_TurnOnLED(ReadButton);
    HAL_Delay(300);
    AML_DebugDevice_TurnOffLED(ReadButton);
  }
}

////////////////////////////////////////////////////////////////////////////////////

void Run(int InitDirection)
{

  // AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
  // AML_DebugDevice_BuzzerBeep(100);
  // HAL_Delay(100);
  // AML_DebugDevice_BuzzerBeep(100);

  // ReadButton = 8;

  // while (ReadButton == 8)
  // {
  // }

  // algorithm = ReadButton; // huong di uu tien

  // for (int i = 0; i < 3; i++)
  // {
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
  //   AML_DebugDevice_BuzzerBeep(150);
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
  //   HAL_Delay(150);
  // }

  // AML_MPUSensor_ResetAngle();
  // HAL_Delay(1500);

  // // algorithm = wallFavor();                 // thay bang ham doc laser
  // // algorithm = AML_LaserSensor_WallFavor(); // can sua lai

  // // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  // // HAL_Delay(1000);

  // // initialize all the structs that we need for this to work
  // // change this coordinate for testing of different
  // // targets for floodfill
  // struct coor target;
  // init_coor(&target, 8, 7);

  // // to flood to center set third parameter to 1
  // init_distance_maze(&distances, &target, 1);

  // // initialize the walls
  // init_wall_maze(&cell_walls_info);

  // update_stack.index = 0;

  // int direction = InitDirection;

  // if (direction == NORTH)
  // {
  //   // if (AML_LaserSensor_ReadSingleWithoutFillter(BR) < WALL_IN_RIGHT)
  //   // {
  //   //   cell_walls_info.cells[0][0].walls[EAST] = 1;
  //   // }
  //   // else
  //   // {
  //   //   cell_walls_info.cells[0][0].walls[EAST] = 0;
  //   // }

  //   cell_walls_info.cells[0][0].walls[EAST] = 1;
  // }

  // if (direction == EAST)
  // {
  //   // if (AML_LaserSensor_ReadSingleWithoutFillter(BL) < WALL_IN_LEFT)
  //   // {
  //   //   cell_walls_info.cells[0][0].walls[NORTH] = 1;
  //   // }
  //   // else
  //   // {
  //   //   cell_walls_info.cells[0][0].walls[NORTH] = 0;
  //   // }

  //   cell_walls_info.cells[0][0].walls[NORTH] = 1;
  // }
  // // set east, south, west wall of start cell to true

  // cell_walls_info.cells[0][0].walls[SOUTH] = 1;
  // cell_walls_info.cells[0][0].walls[WEST] = 1;

  // // cell_walls_info.cells[0][0].visited = 1;

  // struct coor c;
  // init_coor(&c, 0, 0);

  // direction = floodFill1(&distances, &c, &cell_walls_info, algorithm, direction, &update_stack);

  // advanceOneCellVisited();
  // advanceOneCellVisited();
  // advanceOneCellVisited();

  // AML_MotorControl_ShortBreak('F');

  // for (int i = 0; i < 20; i++)
  // {
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
  //   AML_DebugDevice_BuzzerBeep(50);
  //   HAL_Delay(100);
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
  //   HAL_Delay(100);
  // }

  // ////////////////////////////////////////////

  // AML_MotorControl_Stop();
  // return;

  // /////////////////////////////////////////////////

  // // it has some problems

  // direction = centerMovement(&cell_walls_info, &c, direction);

  // // ONCE IT REACHES HERE, IT HAS REACHED THE CENTER OF THE MAZE
  // // Mouse has made it to center, so flood back to start
  // init_coor(&target, 0, 0);
  // init_distance_maze(&distances, &target, 0);

  // // for (int i = 0; i < 10; i++)
  // // {
  // //   AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
  // //   AML_DebugDevice_BuzzerBeep(50);
  // //   HAL_Delay(100);
  // //   AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
  // //   HAL_Delay(100);
  // // }

  // // center to start
  // logicalFlood(&distances, &c, &cell_walls_info, direction, direction, &update_stack);
  // direction = floodFill1(&distances, &c, &cell_walls_info, direction, direction, &update_stack);
  // int difference = direction - NORTH;
  // switch (difference)
  // {
  // case -3:
  //   // leftStillTurn();
  //   // AML_MotorControl_LeftStillTurn();
  //   AML_MotorControl_TurnLeft90();

  //   break;
  // case -2:
  //   // backward180StillTurn();
  //   // AML_MotorControl_BackStillTurn();
  //   AML_MotorControl_TurnLeft180();
  //   break;
  // case -1:
  //   // rightStillTurn();
  //   // AML_MotorControl_RightStillTurn();
  //   AML_MotorControl_TurnRight90();
  //   break;
  // case 0:
  //   break;
  // case 1:
  //   // leftStillTurn();
  //   // AML_MotorControl_LeftStillTurn();
  //   AML_MotorControl_TurnLeft90();
  //   break;
  // case 2:
  //   // backward180StillTurn();
  //   // AML_MotorControl_BackStillTurn();
  //   AML_MotorControl_TurnLeft180();
  //   break;
  // case 3:
  //   // rightStillTurn();
  //   // AML_MotorControl_RightStillTurn();
  //   AML_MotorControl_TurnRight90();
  //   break;
  // default:
  //   // turnOnLEDS();
  //   break;
  // }

  // direction = NORTH;
  // // start to center in "shortest path"
  // init_distance_maze(&distances, &c, 1);
  // logicalFlood(&distances, &c, &cell_walls_info, direction, direction, &update_stack);

  // // lockInterruptDisable_TIM3();
  // AML_MotorControl_TurnOffWallFollow();

  // // leftMotorPWMChangeBackward(200);
  // // rightMotorPWMChangeBackward(200);

  // // custom_delay(2000);
  // AML_MotorControl_LeftPWM(-20);
  // AML_MotorControl_RightPWM(-20);
  // HAL_Delay(2000);
  // AML_MPUSensor_ResetAngle();
  // AML_Encoder_ResetLeftValue();
  // AML_Encoder_ResetRightValue();

  // // motorStop();
  // AML_MotorControl_Stop();
  // // HAL_Delay(100);

  // // wallFavor();

  // for (int i = 0; i < 5; i++)
  // {
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
  //   AML_DebugDevice_BuzzerBeep(50);
  //   HAL_Delay(100);
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
  //   HAL_Delay(100);
  // }

  // return;

  // // custom_delay(1000);

  // // cell_walls_info.cells[0][0].visited = 1;

  // shortestPath(&distances, &c, &cell_walls_info, direction, direction, &update_stack);

  // for (int i = 0; i < 5; i++)
  // {
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
  //   AML_DebugDevice_BuzzerBeep(50);
  //   HAL_Delay(100);
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
  //   HAL_Delay(100);
  // }

  // // motorStop();
  // AML_MotorControl_Stop();

  // // turnOnLEDS();

  // // HAL_Delay(3000);
}

void ShortestPath()
{
  // for (int i = 0; i < 5; i++)
  // {
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
  //   AML_DebugDevice_BuzzerBeep(50);
  //   HAL_Delay(100);
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
  //   HAL_Delay(100);
  // }

  // AML_MPUSensor_ResetAngle();
  // AML_MotorControl_ResetTempSetpoint();

  // int direction = NORTH;
  // struct coor c;
  // init_coor(&c, 0, 0);

  // logicalFlood(&distances, &c, &cell_walls_info, direction, direction, &update_stack);

  // HAL_Delay(1000);

  // shortestPath(&distances, &c, &cell_walls_info, direction, direction, &update_stack);

  // for (int i = 0; i < 20; i++)
  // {
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_SET);
  //   AML_DebugDevice_BuzzerBeep(50);
  //   HAL_Delay(100);
  //   AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
  //   HAL_Delay(100);
  // }

  // return;
}

void RunMode()
{

  // AML_DebugDevice_BuzzerBeep(20);
  // AML_DebugDevice_SetAllLED(GPIO_PIN_SET);

  // HAL_Delay(100);

  // AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);

  // HAL_Delay(100);

  // AML_DebugDevice_SetAllLED(GPIO_PIN_SET);

  // HAL_Delay(200);

  // ReadButton = 8;

  // uint32_t now = HAL_GetTick();

  // while (ReadButton == 8)
  // {
  //   if (HAL_GetTick() - now > 150)
  //   {
  //     AML_DebugDevice_ToggleAllLED();
  //     now = HAL_GetTick();
  //   }
  // }

  // if (ReadButton == 0)
  // {
  //   Run(NORTH);
  // }
  // else if (ReadButton == 1)
  // {
  //   Run(EAST);
  // }
}

void RunNewAlgorithm()
{
  // debug_log("Running...");
  initialize();

  // start the search
  searchRun();

  // reached the center, now calculate the shortest path
  markCenterWall();
  calculateShortestPathDistances();

  while (ReadButton != 8)
  {
  }
  ReadButton = 8;

  // use hand to move the mouse to the start position, and find the shortest path
  // API_ackReset();
  setPosition(0, 0, NORTH);

  // run the shortest path
  fastRunWithVariableVelocity();

  while (ReadButton == 8)
  {
    // run from the center to the start
    searchCenterToStart();

    // updateDistances();
    calculateShortestPathDistances();

    // run the shortest path
    fastRunWithVariableVelocity();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
