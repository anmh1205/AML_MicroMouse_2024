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

void AML_DebugDevice_TurnOnLED(uint8_t number);
void AML_DebugDevice_TurnOffLED(uint8_t number);
void AML_DebugDevice_ToggleLED(uint8_t number);
void AML_DebugDevice_SetLED(uint8_t number, GPIO_PinState state);
void AML_DebugDevice_SetAllLED(GPIO_PinState state);
void AML_DebugDevice_ToggleAllLED(void);
void AML_DebugDevice_SetOnlyOneLED(uint8_t number);

#endif // AML_DEBUGDEVICE_H
