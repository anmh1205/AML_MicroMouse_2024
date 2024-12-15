#ifndef AML_IRSENSOR_H
#define AML_IRSENSOR_H

#include "stm32h7xx_hal.h"
#include <AML_GlobalVariable.h>
#include <stdbool.h>

void AML_IRSensor_Setup(void);
double AML_IRSensor_GetDistance(uint8_t sensor);

bool AML_IRSensor_IsFrontWall(void);
bool AML_IRSensor_IsLeftWall(void);
bool AML_IRSensor_IsRightWall(void);

bool AML_IRSensor_IsNoFrontWall(void);
bool AML_IRSensor_IsNoLeftWall(void);
bool AML_IRSensor_IsNoRightWall(void);

#endif

