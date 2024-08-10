#include "AML_DebugDevice.h"



uint16_t Led[8] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_10};

uint8_t LedIndex = 0;
uint8_t LedIndexFlag = 1;
uint8_t MidIndex = 0;



void AML_DebugDevice_TurnOnLED(COLOR color)
{
    HAL_GPIO_WritePin(GPIOC, Led[color], GPIO_PIN_SET);
}

void AML_DebugDevice_TurnOffLED(COLOR color)
{
    HAL_GPIO_WritePin(GPIOC, Led[color], GPIO_PIN_RESET);
}

void AML_DebugDevice_ToggleLED(COLOR color)
{
    HAL_GPIO_TogglePin(GPIOC, Led[color]);
}

void AML_DebugDevice_SetLED(COLOR color, GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, Led[color], state);
}

void AML_DebugDevice_SetAllLED(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, Led[0], state);
    HAL_GPIO_WritePin(GPIOC, Led[1], state);
    HAL_GPIO_WritePin(GPIOC, Led[2], state);
    HAL_GPIO_WritePin(GPIOC, Led[3], state);
    HAL_GPIO_WritePin(GPIOC, Led[4], state);
    HAL_GPIO_WritePin(GPIOC, Led[5], state);
    HAL_GPIO_WritePin(GPIOC, Led[6], state);
    HAL_GPIO_WritePin(GPIOC, Led[7], state);
}

void AML_DebugDevice_ToggleAllLED()
{
    HAL_GPIO_TogglePin(GPIOC, Led[0]);
    HAL_GPIO_TogglePin(GPIOC, Led[1]);
    HAL_GPIO_TogglePin(GPIOC, Led[2]);
    HAL_GPIO_TogglePin(GPIOC, Led[3]);
    HAL_GPIO_TogglePin(GPIOC, Led[4]);
    HAL_GPIO_TogglePin(GPIOC, Led[5]);
    HAL_GPIO_TogglePin(GPIOC, Led[6]);
    HAL_GPIO_TogglePin(GPIOC, Led[7]);
}

void AML_DebugDevice_Handle()
{
    // if (LedIndexFlag)
    // {
    //     AML_DebugDevice_ToggleLED(LedIndex++);

    //     if (LedIndex == 8)
    //     {
    //         LedIndex = 7;
    //         LedIndexFlag = 0;
    //     }
    // }
    // else
    // {
    //     AML_DebugDevice_ToggleLED(LedIndex--);

    //     if (LedIndex == 0)
    //     {
    //         LedIndex = 0;
    //         LedIndexFlag = 1;
    //     }
    // }

    // if (LedIndexFlag)
    // {
    //     Led
    // }
    // else
    // {
    //     AML_DebugDevice_TurnOffLED(LedIndex--);

    //     if (LedIndex == 0)
    //     {
    //         LedIndex = 0;
    //         LedIndexFlag = 1;
    //     }
    // }
}

