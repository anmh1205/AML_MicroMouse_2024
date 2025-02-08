#include "AML_LedDebug.h"

// extern TIM_HandleTypeDef htim10;

GPIO_TypeDef *LedPort = GPIOD;
uint16_t Led[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};

uint8_t LedIndex = 0;
uint8_t LedIndexFlag = 1;
uint8_t MidIndex = 0;

//-------------------------------------------------------------------------------------------------------//

void AML_LedDebug_TurnOnLED(COLOR color);
void AML_LedDebug_TurnOffLED(COLOR color);
void AML_LedDebug_ToggleLED(COLOR color);
void AML_LedDebug_SetLED(COLOR color, GPIO_PinState state);
void AML_LedDebug_SetAllLED(GPIO_PinState state);
void AML_LedDebug_ToggleAllLED();
void AML_LedDebug_SetOnlyOneLED(COLOR color);

//-------------------------------------------------------------------------------------------------------//

void AML_LedDebug_TurnOnLED(COLOR color)
{
    HAL_GPIO_WritePin(GPIOD, Led[color], GPIO_PIN_SET);
}

void AML_LedDebug_TurnOffLED(COLOR color)
{
    HAL_GPIO_WritePin(GPIOD, Led[color], GPIO_PIN_RESET);
}

void AML_LedDebug_ToggleLED(COLOR color)
{
    HAL_GPIO_TogglePin(GPIOD, Led[color]);
}

void AML_LedDebug_SetLED(COLOR color, GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOD, Led[color], state);
}

void AML_LedDebug_SetAllLED(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOD, Led[0], state);
    HAL_GPIO_WritePin(GPIOD, Led[1], state);
    HAL_GPIO_WritePin(GPIOD, Led[2], state);
    HAL_GPIO_WritePin(GPIOD, Led[3], state);
}

void AML_LedDebug_ToggleAllLED(void)
{
    HAL_GPIO_TogglePin(GPIOD, Led[0]);
    HAL_GPIO_TogglePin(GPIOD, Led[1]);
    HAL_GPIO_TogglePin(GPIOD, Led[2]);
    HAL_GPIO_TogglePin(GPIOD, Led[3]);
}

void AML_LedDebug_SetOnlyOneLED(COLOR color)
{
    HAL_GPIO_WritePin(GPIOD, Led[0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Led[1], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Led[2], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Led[3], GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOD, Led[color], GPIO_PIN_SET);
}

