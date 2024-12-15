#include "AML_Keyboard.h"

typedef enum
{
    SW1,
    SW2,
    SW3,
    SW4,
    SW5
} SW;

uint16_t Button[] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5};
extern int16_t debug[100];

extern uint8_t ReadButton;
extern uint8_t ButtonPressed;

void AML_Keyboard_Setup(void)
{
}

GPIO_PinState AML_Keyboard_GetKey(uint8_t key)
{
    return HAL_GPIO_ReadPin(ButtonPORT, Button[key]);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    UNUSED(GPIO_Pin);

    if (GPIO_Pin == Button[SW1])
    {
        ReadButton = 0;
    }
    else if (GPIO_Pin == Button[SW2])
    {
        ReadButton = 1;
    }
    else if (GPIO_Pin == Button[SW3])
    {
        ReadButton = 2;
        // AML_MotorControl_TurnOnWallFollow();
    }
    else if (GPIO_Pin == Button[SW4])
    {
        ReadButton = 3;
        // AML_MotorControl_TurnOffWallFollow();
    }
    else if (GPIO_Pin == Button[SW5])
    {
        ReadButton = 4;
    }

    ButtonPressed = 1;
}
