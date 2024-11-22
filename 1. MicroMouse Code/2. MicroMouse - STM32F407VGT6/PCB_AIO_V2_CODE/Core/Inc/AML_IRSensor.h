#ifndef AML_IRSENSOR_H
#define AML_IRSENSOR_H

#include "stm32h7xx_hal.h"
#include <AML_GlobalVariable.h>

void AML_IRSensor_Setup(void);
double AML_IRSensor_GetDistance(uint8_t sensor);

uint8_t AML_IRSensor_IsFrontWall(void);
uint8_t AML_IRSensor_IsLeftWall(void);
uint8_t AML_IRSensor_IsRightWall(void);

#endif