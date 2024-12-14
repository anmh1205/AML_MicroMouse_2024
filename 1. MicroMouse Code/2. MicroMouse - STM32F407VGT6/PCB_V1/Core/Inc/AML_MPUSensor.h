#ifndef AML_MPUSensor_H
#define AML_MPUSensor_H

#include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_uart.h"
#include "pid.h"
#include "AML_Remote.h"

void AML_MPUSensor_Setup(void);
void AML_MPUSensor_ResetAngle(void);
double AML_MPUSensor_GetAngle(void);

#endif /* test_h */
