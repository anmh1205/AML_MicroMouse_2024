#ifndef AML_KEYBOARD_H
#define AML_KEYBOARD_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "AML_MotorControl.h"
#include "AML_DebugDevice.h"

// Button number
#define ButtonPORT GPIOE



GPIO_PinState AML_Keyboard_GetKey(uint8_t key);



#endif // AML_KEYBOARD_H
