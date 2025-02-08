#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
#include "AML_LaserSensor.h"
#include "AML_Encoder.h"
#include "AML_MPUSensor.h"
#include "AML_Parameter.h"
#include "AML_PID.h"

#include "pid.h"
#include "math.h"
#include <stdint.h>
#include <limits.h>
// #include "math.h"

void AML_MotorControl_Setup(void);

void AML_MotorControl_PIDSetTunnings(double Kp, double Ki, double Kd);

void AML_MotorControl_LeftPWM(int32_t PWMValue);
void AML_MotorControl_RightPWM(int32_t PWMValue);

void AML_MotorControl_SetDirection(GPIO_PinState dir);
void AML_MotorControl_SetMouseSpeed(int32_t speed);
void AML_MotorControl_ResetTempSetpoint(void);

void AML_MotorControl_SetLeftSpeed(double speed, GPIO_PinState direction);
void AML_MotorControl_SetRightSpeed(double speed, GPIO_PinState direction);

void AML_MotorControl_ShortBreak(char c);
void AML_MotorControl_Stop(void);

void AML_MotorControl_SetCenterPosition(void);
void AML_MotorControl_SetLeftWallValue(void);
void AML_MotorControl_SetRightWallValue(void);

void AML_MotorControl_LeftWallFollow(void);
void AML_MotorControl_RightWallFollow(void);
void AML_MotorControl_MPUFollow(double setpoint);
void AML_MotorControl_GoStraight(void);
void AML_MotorControl_TurnOnWallFollow(void);
void AML_MotorControl_TurnOffWallFollow(void);

void AML_MotorControl_AdvanceTicks(int16_t ticks);
void AML_MotorControl_MoveForward(int16_t distance, uint8_t speed);
void AML_MotorControl_TurnLeft90(void);
void AML_MotorControl_TurnRight90(void);
void AML_MotorControl_TurnLeft180(void);
void AML_MotorControl_TurnRight180(void);

void AML_MotorControl_ResetTempSetpoint(void);

void AML_MotorControl_LeftStillTurn(void);
void AML_MotorControl_RightStillTurn(void);
void AML_MotorControl_BackStillTurn(void);

void AML_MotorControl_MoveForward_mm(uint16_t distance);


#endif // AML_MOTORCONTROL_H