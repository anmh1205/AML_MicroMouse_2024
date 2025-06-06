#ifndef AML_SWITCH_H
#define AML_SWITCH_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "AML_GlobalVariable.h"
#include <stdbool.h>

void AML_ReadAll_BitSwitch(void);
bool AML_Read_BitSwitch(uint8_t index);

void AML_ReadAll_Button(void);
bool AML_Read_Button(uint8_t index);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif

