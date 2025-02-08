#ifndef AML_DEBUGDEVICE_H
#define AML_DEBUGDEVICE_H

#include "stm32f4xx_hal.h"
#include "main.h"

// Led number
// #define LEDPORT GPIOC

typedef enum
{
    WHITE,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    ORANGE,
    PURPLE,
    PINK
} COLOR;

void AML_DebugDevice_BuzzerBeep(uint16_t delay);

void AML_DebugDevice_TurnOnLED(COLOR color);
void AML_DebugDevice_TurnOffLED(COLOR color);
void AML_DebugDevice_ToggleLED(COLOR color);
void AML_DebugDevice_SetLED(COLOR color, GPIO_PinState state);
void AML_DebugDevice_SetAllLED(GPIO_PinState state);
void AML_DebugDevice_ToggleAllLED();
void AML_DebugDevice_SetOnlyOneLED(COLOR color);

#endif // AML_DEBUGDEVICE_H
