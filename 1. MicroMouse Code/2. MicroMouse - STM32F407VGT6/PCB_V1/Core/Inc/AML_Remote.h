#ifndef AML_REMOTE_H
#define AML_REMOTE_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "AML_MotorControl.h"
#include "AML_DebugDevice.h"
#include "pid.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

void AML_Remote_Setup(void);
void AML_Remote_Handle(void);

#endif // AML_REMOTE_H
