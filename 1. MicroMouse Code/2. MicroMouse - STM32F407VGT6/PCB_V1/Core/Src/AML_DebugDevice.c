#include "AML_DebugDevice.h"

extern TIM_HandleTypeDef htim10;

uint16_t Led[8] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_10};

uint8_t LedIndex = 0;
uint8_t LedIndexFlag = 1;
uint8_t MidIndex = 0;

void AML_DebugDevice_BuzzerBeep(uint16_t delay)
{
    uint32_t InitTime = HAL_GetTick();

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

    while (HAL_GetTick() - InitTime < (uint32_t)delay)
        ;

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

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
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(GPIOC, Led[i], state);
    }
}

void AML_DebugDevice_ToggleAllLED()
{
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_TogglePin(GPIOC, Led[i]);
    }
}

void AML_DebugDevice_SetOnlyOneLED(COLOR color)
{
    AML_DebugDevice_SetAllLED(GPIO_PIN_RESET);
    AML_DebugDevice_SetLED(color, GPIO_PIN_SET);
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

void AML_DebugDevice_TurnOnIT()
{
    HAL_TIM_Base_Start_IT(&htim10);
}

void AML_DebugDevice_TurnOffIT()
{
    HAL_TIM_Base_Stop_IT(&htim10);
}
