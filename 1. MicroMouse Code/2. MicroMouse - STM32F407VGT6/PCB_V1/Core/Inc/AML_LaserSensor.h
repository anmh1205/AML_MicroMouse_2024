#ifndef AML_LASERSENSOR_H
#define AML_LASERSENSOR_H

#include "stm32f4xx.h"
#include "vl53l0x_api.h"
#include "main.h"
#include "KalmanFilter.h"
#include "AML_Remote.h"
#include "AML_Parameter.h"



typedef enum
{
    FL,
    FF,
    FR,
    BR,
    BL
} LaserName;

void AML_LaserSensor_Setup(void);
void AML_LaserSensor_ReadAll(void);
int32_t AML_LaserSensor_ReadSingleWithFillter(uint8_t name);
int32_t AML_LaserSensor_ReadSingleWithoutFillter(uint8_t name);

uint8_t AML_LaserSensor_IsLeftWall(void);
uint8_t AML_LaserSensor_IsFrontWall(void);
uint8_t AML_LaserSensor_IsRightWall(void);

#endif // AML_LASERSENSOR_H
