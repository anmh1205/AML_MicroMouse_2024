#include "AML_MotorControl.h"

#define Pi 3.14159265359  // Pi number
#define WheelDiameter 50  // mm
#define Ratio 90 / 14     // 90:14
#define PulsePerRound 190 // 190 pulse per round encoder
int32_t MouseSpeed = 100; // % PWM

uint8_t FinishFlag;
uint8_t TurnFlag;
uint16_t PreviousBL, PreviousBR;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;

extern volatile uint8_t RemarkAfterTurnMode;

extern int16_t debug[100];
extern uint8_t ReadButton;
extern VL53L0X_RangingMeasurementData_t SensorValue[7];
// int32_t LeftValue, RightValue;
// int32_t PreviousLeftEncoderValue = 0, PreviousRightEncoderValue = 0;

double SetpointAngle = 0, TempSetpoint = 0;

GPIO_PinState direction = GPIO_PIN_SET;

double Input_Left, Output_Left, Setpoint_Left;
double Input_Right, Output_Right, Setpoint_Right;

PID_TypeDef PID_SpeedLeft;
double Speed_Kp_Left = 1.0f;
double Speed_Ki_Left = 0.1f;
double Speed_Kd_Left = 0.1f;
double PreviouseLeftEncoderValue = 0;

PID_TypeDef PID_SpeedRight;
double Speed_Kp_Right = 1.0f;
double Speed_Ki_Right = 0.0f;
double Speed_Kd_Right = 0.f;
double PreviouseRightEncoderValue = 0;

PID_TypeDef PID_PositionLeft;
double Position_Kp = 0.005f;
double Position_Ki = 0.007f;
double Position_Kd = 0.0f;

PID_TypeDef PID_PositionRight;

double SetpointDistance;

///////////////////////////////////

PID_TypeDef PID_LeftWallFollow;
double LeftWallDistance;
double LeftWallFollow_Kp = 0.25f;
double LeftWallFollow_Ki = 0.03f;
double LeftWallFollow_Kd = 0.4f;

PID_TypeDef PID_RightWallFollow;
double RightWallDistance;
double RightWallFollow_Kp = 0.25f;
double RightWallFollow_Ki = 0.03f;
double RightWallFollow_Kd = 0.4f;

PID_TypeDef PID_MPUFollow;
double Input_MPUFollow, Output_MPUFollow, Setpoint_MPUFollow;
double MPUFollow_Kp = 0.22f;
double MPUFollow_Ki = 0.01f;
double MPUFollow_Kd = 0.1f;

//////////////////////////////////

double MinRotateSpeed = -20;
double MaxRotateSpeed = 20;

double TurnLeftAngle = 98;
double TurnRightAngle = -86;
double BackwardLeftAngle = 195;
double BackwardRightAngle = -177;

// double TurnLeftAngle = 90;
// double TurnRightAngle = -90;
// double BackwardLeftAngle = 180;
// double BackwardRightAngle = -180;

PID_TypeDef PID_TurnLeft;
double Input_TurnLeft, Output_TurnLeft, Setpoint_TurnLeft;
double TurnLeft_Kp = 1.0;
double TurnLeft_Ki = 1.2;
double TurnLeft_Kd = 0.35;

PID_TypeDef PID_TurnRight;
double Input_TurnRight, Output_TurnRight, Setpoint_TurnRight;
double TurnRight_Kp = 0.9;
double TurnRight_Ki = 1.1;
double TurnRight_Kd = 0.35;

// AML_PID_Struct AML_PID_TurnLeft;
// double AML_PID_TurnLeft_Kp = 0.25;
// double AML_PID_TurnLeft_Ki = 0.67;
// double AML_PID_TurnLeft_Kd = 0.3;
// double AML_PID_TurnLeft_tau = 1.0;

// AML_PID_Struct AML_PID_TurnRight;
// double AML_PID_TurnRight_Kp = 0.25;
// double AML_PID_TurnRight_Ki = 0.67;
// double AML_PID_TurnRight_Kd = 0.3;
// double AML_PID_TurnRight_tau = 1.0;

AML_PID_Struct AML_PID_TurnLeft =
    {
        .Kp = 0.25,
        .Ki = 0.9,
        .Kd = 0.3,
        .tau = 1.0,
        .limMax = 20,
        .limMin = -20,
        .linMaxInt = 7,
        .linMinInt = -7,
        .lastTime = 0,
        .sampleTime = 20,
        .prevError = 0,
        .integratol = 0,
        .prevMeasurement = 0,
        .differentiator = 0,
        .Output = 0,
        .Setpoint = 0,
};

AML_PID_Struct AML_PID_TurnRight =
    {
        .Kp = 0.25,
        .Ki = 0.9,
        .Kd = 0.3,
        .tau = 1.0,
        .limMax = 20,
        .limMin = -20,
        .linMaxInt = 7,
        .linMinInt = -7,
        .lastTime = 0,
        .sampleTime = 20,
        .prevError = 0,
        .integratol = 0,
        .prevMeasurement = 0,
        .differentiator = 0,
        .Output = 0,
        .Setpoint = 0,
};

AML_PID_Struct PID_LeftMotor =
    {
        .Kp = 0.1,
        .Ki = 2,
        .Kd = 0,
        .tau = 0.1,
        .limMin = 0,
        .limMax = 50,
        .linMinInt = 0,
        .linMaxInt = 25,
        .sampleTime = 20,
        .lastTime = 0,
        .integratol = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevMeasurement = 0,
        .Input = 0,
        .Output = 0,
        .Setpoint = 0,
};

////////////////////////////////////////

int32_t MinLeftWallDistance = 93;

uint16_t MaxLeftWallDistance;

int32_t MinRightWallDistance = 73;

uint16_t MaxRightWallDistance;
uint16_t MinFrontWallDistance;

double OutputMin = 0;
double OutputMax = 70;

uint8_t ResetMPUFlag = 0;

// int64_t PreviousTime = 0;

void AML_MotorControl_LeftMotorSpeed(int32_t rpm);

///////////////////////////////////////////////////////////////////////////////////////

double ABS(double value)
{
    return value > 0 ? value : -value;
}

int32_t double_to_int32_t(double value)
{
    if (value < INT32_MIN)
    {
        return INT32_MIN;
    }
    else if (value > INT32_MAX)
    {
        return INT32_MAX;
    }
    else
    {
        return (int32_t)value;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void AML_MotorControl_PIDSetTunings(double Kp, double Ki, double Kd)
// {
//     PID_SpeedLeft.Kp = Kp;
//     PID_SpeedLeft.Ki = Ki;
//     PID_SpeedLeft.Kd = Kd;

//     PID_SpeedRight.Kp = Kp;
//     PID_SpeedRight.Ki = Ki;
//     PID_SpeedRight.Kd = Kd;
// }

void AML_MotorControl_PIDSetTunnings(double Kp, double Ki, double Kd)
{
    PID_TypeDef *pid = &PID_LeftWallFollow;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void AML_MotorControl_PIDSetSampleTime(uint32_t NewSampleTime)
{
    PID_SpeedLeft.SampleTime = NewSampleTime;
    PID_SpeedRight.SampleTime = NewSampleTime;

    PID_PositionLeft.SampleTime = NewSampleTime;
    PID_PositionRight.SampleTime = NewSampleTime;

    PID_LeftWallFollow.SampleTime = NewSampleTime;
    PID_RightWallFollow.SampleTime = NewSampleTime;

    PID_TurnLeft.SampleTime = NewSampleTime;
    PID_TurnRight.SampleTime = NewSampleTime;

    PID_MPUFollow.SampleTime = NewSampleTime;
}

void AML_MotorControl_PIDSetOutputLimits(double Min, double Max)
{
    PID_SpeedLeft.OutMin = Min;
    PID_SpeedLeft.OutMax = Max;

    PID_SpeedRight.OutMin = Min;
    PID_SpeedRight.OutMax = Max;

    PID_PositionLeft.OutMin = -20;
    PID_PositionLeft.OutMax = 20;

    PID_PositionRight.OutMin = Min;
    PID_PositionRight.OutMax = Max;

    PID_LeftWallFollow.OutMin = -15;
    PID_LeftWallFollow.OutMax = 15;

    PID_RightWallFollow.OutMin = -15;
    PID_RightWallFollow.OutMax = 15;

    PID_TurnLeft.OutMin = MinRotateSpeed;
    PID_TurnLeft.OutMax = MaxRotateSpeed;

    PID_TurnRight.OutMin = MinRotateSpeed;
    PID_TurnRight.OutMax = MaxRotateSpeed;

    PID_MPUFollow.OutMin = -15;
    PID_MPUFollow.OutMax = 15;
}

void AML_MotorControl_PIDSetMode(PIDMode_TypeDef Mode)
{
    PID_SetMode(&PID_SpeedLeft, Mode);
    PID_SetMode(&PID_SpeedRight, Mode);

    PID_SetMode(&PID_PositionLeft, Mode);
    PID_SetMode(&PID_PositionRight, Mode);

    PID_SetMode(&PID_LeftWallFollow, Mode);
    PID_SetMode(&PID_RightWallFollow, Mode);

    PID_SetMode(&PID_TurnLeft, Mode);
    PID_SetMode(&PID_TurnRight, Mode);

    PID_SetMode(&PID_MPUFollow, Mode);
}

void AML_MotorControl_PIDSetup()
{
    PID_Init(&PID_SpeedLeft);
    PID_Init(&PID_SpeedRight);

    PID_Init(&PID_PositionLeft);
    PID_Init(&PID_PositionRight);

    PID_Init(&PID_LeftWallFollow);
    PID_Init(&PID_RightWallFollow);

    PID_Init(&PID_TurnLeft);
    PID_Init(&PID_TurnRight);

    PID_Init(&PID_MPUFollow);

    PID(&PID_SpeedLeft, &Input_Left, &Output_Left, &Setpoint_Left, Speed_Kp_Left, Speed_Ki_Left, Speed_Kd_Left, _PID_P_ON_E, PID_SpeedLeft.ControllerDirection);
    PID(&PID_SpeedRight, &Input_Right, &Output_Right, &Setpoint_Right, Speed_Kp_Right, Speed_Ki_Right, Speed_Kd_Right, _PID_P_ON_E, PID_SpeedRight.ControllerDirection);

    PID(&PID_PositionLeft, &Input_Left, &Output_Left, &Setpoint_Left, Position_Kp, Position_Ki, Position_Kd, _PID_P_ON_E, PID_PositionLeft.ControllerDirection);
    PID(&PID_PositionRight, &Input_Right, &Output_Right, &Setpoint_Right, Position_Kp, Position_Ki, Position_Kd, _PID_P_ON_E, PID_PositionRight.ControllerDirection);

    PID(&PID_LeftWallFollow, &LeftWallDistance, &Output_Left, &SetpointDistance, LeftWallFollow_Kp, LeftWallFollow_Ki, LeftWallFollow_Kd, _PID_P_ON_E, PID_LeftWallFollow.ControllerDirection);
    PID(&PID_RightWallFollow, &RightWallDistance, &Output_Right, &SetpointDistance, RightWallFollow_Kp, RightWallFollow_Ki, RightWallFollow_Kd, _PID_P_ON_E, PID_RightWallFollow.ControllerDirection);

    PID(&PID_TurnLeft, &Input_TurnLeft, &Output_TurnLeft, &Setpoint_TurnLeft, TurnLeft_Kp, TurnLeft_Ki, TurnLeft_Kd, _PID_P_ON_E, PID_TurnLeft.ControllerDirection);
    PID(&PID_TurnRight, &Input_TurnRight, &Output_TurnRight, &Setpoint_TurnRight, TurnRight_Kp, TurnRight_Ki, TurnRight_Kd, _PID_P_ON_E, PID_TurnRight.ControllerDirection);

    PID(&PID_MPUFollow, &Input_MPUFollow, &Output_MPUFollow, &Setpoint_MPUFollow, MPUFollow_Kp, MPUFollow_Ki, MPUFollow_Kd, _PID_P_ON_E, PID_MPUFollow.ControllerDirection);

    // AML_MotorControl_PIDSetTunings();

    AML_MotorControl_PIDSetSampleTime(20);
    AML_MotorControl_PIDSetOutputLimits(OutputMin, OutputMax);
    AML_MotorControl_PIDSetMode(_PID_MODE_AUTOMATIC);
}

void AML_MotorControl_PIDReset(PID_TypeDef *uPID)
{
    // uPID->MyInput = 0;
    // uPID->MyOutput = 0;
    // uPID->MySetpoint = 0;

    // uPID->OutputSum = 0;
    // uPID->LastInput = 0;
    // uPID->LastTime = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AML_MotorControl_AMLPIDSetOutputLimits(double Min, double Max)
{
    // AML_PID_TurnLeft.limMin = Min;
    // AML_PID_TurnLeft.limMax = Max;

    // AML_PID_TurnLeft.linMinInt = -9;
    // AML_PID_TurnLeft.linMaxInt = 9;

    // AML_PID_TurnRight.limMin = Min;
    // AML_PID_TurnRight.limMax = Max;

    // AML_PID_TurnRight.linMinInt = -9;
    // AML_PID_TurnRight.linMaxInt = 9;
}

void AML_MotorControl_AMLPIDSetup()
{
    // AML_PID_Init(&AML_PID_TurnLeft, AML_PID_TurnLeft_Kp, AML_PID_TurnLeft_Ki, AML_PID_TurnLeft_Kd, AML_PID_TurnLeft_tau, 20);
    // AML_PID_Init(&AML_PID_TurnRight, AML_PID_TurnRight_Kp, AML_PID_TurnRight_Ki, AML_PID_TurnRight_Kd, AML_PID_TurnRight_tau, 20);

    // AML_MotorControl_AMLPIDSetOutputLimits(MinRotateSpeed, MaxRotateSpeed);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// init motor control
void AML_MotorControl_Setup(void)
{
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    AML_MotorControl_PIDSetup();
    // AML_MotorControl_AMLPIDSetup();

    TurnFlag = 0;
    FinishFlag = 0;
}

void AML_MotorControl_LeftPWM(int32_t PWMValue)
{
    if (PWMValue > 100)
    {
        PWMValue = 100;
    }
    else if (PWMValue < -100)
    {
        PWMValue = -100;
    }

    if (PWMValue > 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, direction);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, !direction);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWMValue);
    }
    else if (PWMValue < 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, !direction);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, direction);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -PWMValue);
    }
    else if (PWMValue == 0)
    {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }
}

void AML_MotorControl_RightPWM(int32_t PWMValue)
{
    PWMValue = (int32_t)(PWMValue * 1);

    if (PWMValue > 100)
    {
        PWMValue = 100;
    }
    else if (PWMValue < -100)
    {
        PWMValue = -100;
    }

    if (PWMValue > 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, !direction);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, direction);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWMValue);
    }
    else if (PWMValue < 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, direction);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, !direction);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -PWMValue);
    }
    else if (PWMValue == 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

void AML_MotorControl_SetDirection(GPIO_PinState dir)
{
    direction = dir;
}

void AML_MotorControl_ToggleDirection(void)
{
    direction = ~direction;
}

void AML_MotorControl_SetMouseSpeed(int32_t speed)
{
    MouseSpeed = speed;
}

void AML_MotorControl_ResetTempSetpoint(void)
{
    TempSetpoint = 0;
}

void AML_MotorControl_SetLeftSpeed(double speed, GPIO_PinState direction)
{
    Setpoint_Left = speed * Ratio;

    double CurrentValue = (double)AML_Encoder_GetLeftValue();
    Input_Left = ABS((double)(CurrentValue - PreviouseLeftEncoderValue) / PulsePerRound);

    PreviouseLeftEncoderValue = CurrentValue;

    PID_Compute(&PID_SpeedLeft);

    // AML_MotorControl_LeftPWM(Output_Left * direction);
}

void AML_MotorControl_SetRightSpeed(double speed, GPIO_PinState direction)
{
    Setpoint_Right = speed * Ratio;
    Input_Right = ABS((double)AML_Encoder_GetRightValue() / PulsePerRound);
    // AML_Encoder_ResetRightValue();

    PID_Compute(&PID_SpeedRight);

    AML_MotorControl_RightPWM(Output_Right * direction);
}

void AML_MotorControl_Stop(void)
{
    AML_MotorControl_LeftPWM(0);
    AML_MotorControl_RightPWM(0);
}

void AML_MotorControl_ShortBreak(char c)
{
    uint8_t TurnSpeed = 30;
    uint8_t GoStraightSpeed = 25;

    if (c == 'L')
    {

        AML_MotorControl_LeftPWM(TurnSpeed);
        AML_MotorControl_RightPWM(-TurnSpeed);
    }
    else if (c == 'R')
    {
        AML_MotorControl_LeftPWM(-TurnSpeed);
        AML_MotorControl_RightPWM(TurnSpeed);
    }
    else if (c == 'F')
    {
        AML_MotorControl_LeftPWM(-GoStraightSpeed);
        AML_MotorControl_RightPWM(-GoStraightSpeed);
    }
    else if (c == 'B')
    {
        AML_MotorControl_LeftPWM(GoStraightSpeed);
        AML_MotorControl_RightPWM(GoStraightSpeed);
    }

    HAL_Delay(30);
    AML_MotorControl_Stop();
}

void AML_MotorControl_SetCenterPosition(void)
{
    AML_LaserSensor_ReadAll();

    MinLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MinRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);

    HAL_Delay(22);

    AML_LaserSensor_ReadAll();

    MinLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MinRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);

    HAL_Delay(22);

    AML_LaserSensor_ReadAll();

    MinLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MinRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);
}

void AML_MotorControl_SetLeftWallValue(void)
{
    MinLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MaxRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);

    MinFrontWallDistance = AML_LaserSensor_ReadSingleWithFillter(FF);

    HAL_Delay(22);

    MinLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MaxRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);

    MinFrontWallDistance = AML_LaserSensor_ReadSingleWithFillter(FF);

    HAL_Delay(22);

    MinLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MaxRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);

    MinFrontWallDistance = AML_LaserSensor_ReadSingleWithFillter(FF);
}

void AML_MotorControl_SetRightWallValue(void)
{
    MaxLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MinRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);

    MinFrontWallDistance = AML_LaserSensor_ReadSingleWithFillter(FF);

    HAL_Delay(22);

    MaxLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MinRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);

    MinFrontWallDistance = AML_LaserSensor_ReadSingleWithFillter(FF);

    HAL_Delay(22);

    MaxLeftWallDistance = AML_LaserSensor_ReadSingleWithFillter(BL);
    MinRightWallDistance = AML_LaserSensor_ReadSingleWithFillter(BR);

    MinFrontWallDistance = AML_LaserSensor_ReadSingleWithFillter(FF);
}

void AML_MotorControl_LeftWallFollow(void)
{
    LeftWallDistance = (double)AML_LaserSensor_ReadSingleWithFillter(BL);

    // SetpointDistance = (double)(MinLeftWallDistance) + 35;
    SetpointDistance = (double)(MinLeftWallDistance);

    PID_Compute(&PID_LeftWallFollow);

    // AML_MotorControl_LeftPWM((MouseSpeed + (int32_t)Output_Left));
    // AML_MotorControl_RightPWM((MouseSpeed - (int32_t)Output_Left));
}

void AML_MotorControl_RightWallFollow(void)
{
    RightWallDistance = (double)AML_LaserSensor_ReadSingleWithFillter(BR);

    // SetpointDistance = (double)(MinRightWallDistance) + 35;
    SetpointDistance = (double)(MinRightWallDistance);

    PID_Compute(&PID_RightWallFollow);

    // AML_MotorControl_LeftPWM((MouseSpeed - (int32_t)Output_Right));
    // AML_MotorControl_RightPWM((MouseSpeed + (int32_t)Output_Right));
}

void AML_MotorControl_MPUFollow(double setpoint)
{
    Setpoint_MPUFollow = setpoint;

    Input_MPUFollow = (double)AML_MPUSensor_GetAngle();

    PID_Compute(&PID_MPUFollow);

    AML_MotorControl_LeftPWM((MouseSpeed - (int32_t)Output_MPUFollow));
    AML_MotorControl_RightPWM((MouseSpeed + (int32_t)Output_MPUFollow));
}

void AML_MotorControl_GoStraight(void)
{
    if (AML_LaserSensor_ReadSingleWithFillter(BL) < WALL_IN_LEFT)
    {
        AML_MotorControl_LeftWallFollow();
    }
    else if (AML_LaserSensor_ReadSingleWithFillter(BR) < WALL_IN_RIGHT)
    {
        AML_MotorControl_RightWallFollow();
    }
    else if (AML_LaserSensor_ReadSingleWithFillter(BL) > WALL_NOT_IN_FRONT_LEFT && AML_LaserSensor_ReadSingleWithFillter(BR) > WALL_NOT_IN_FRONT_RIGHT)
    {
        AML_MotorControl_MPUFollow(0);
    }
}

void AML_MotorControl_GoStraight2(void)
{
    if (AML_LaserSensor_ReadSingleWithoutFillter(BL) < WALL_IN_LEFT)
    {
        AML_MotorControl_LeftWallFollow();
    }
    else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) < WALL_IN_RIGHT)
    {
        AML_MotorControl_RightWallFollow();
    }
}

void AML_MotorControl_GoStraight3(void)
{
    AML_MotorControl_MPUFollow(SetpointAngle);
}

void AML_MotorControl_GoStraight4(void)
{
    if (AML_LaserSensor_ReadSingleWithFillter(BL) > WALL_IN_LEFT && AML_LaserSensor_ReadSingleWithFillter(BR) > WALL_IN_RIGHT)
    {
        AML_MotorControl_MPUFollow(0);
    }
    else if (AML_LaserSensor_ReadSingleWithFillter(BL) < WALL_IN_LEFT)
    {
        AML_MotorControl_LeftWallFollow();

        TempSetpoint = -*PID_LeftWallFollow.MyOutput;

        AML_MotorControl_MPUFollow(TempSetpoint);
    }
    else if (AML_LaserSensor_ReadSingleWithFillter(BR) < WALL_IN_RIGHT)
    {
        AML_MotorControl_RightWallFollow();

        TempSetpoint = *PID_RightWallFollow.MyOutput;

        AML_MotorControl_MPUFollow(TempSetpoint);
    }
}

void AML_MotoControl_GoStraight5(void)
{
    if (AML_LaserSensor_ReadSingleWithFillter(BL) > WALL_NOT_IN_LEFT && AML_LaserSensor_ReadSingleWithFillter(BR) > WALL_NOT_IN_RIGHT)
    {
        AML_MotorControl_MPUFollow(TempSetpoint);
    }
    else if (AML_LaserSensor_ReadSingleWithoutFillter(BL) < WALL_IN_LEFT)
    {
        AML_MotorControl_LeftWallFollow();

        // TempSetpoint = -*PID_LeftWallFollow.MyOutput;

        AML_MotorControl_MPUFollow(TempSetpoint - *PID_LeftWallFollow.MyOutput);
    }
    else if (AML_LaserSensor_ReadSingleWithoutFillter(BR) < WALL_IN_RIGHT)
    {
        AML_MotorControl_RightWallFollow();

        // TempSetpoint = *PID_RightWallFollow.MyOutput;

        AML_MotorControl_MPUFollow(TempSetpoint + *PID_RightWallFollow.MyOutput);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // UNUSED(htim);
    if (htim->Instance == htim9.Instance) // timer for wall follow
    {
        // AML_LaserSensor_ReadAll();
        // AML_MotorControl_GoStraight();
        // AML_MotorControl_GoStraight2();
        // AML_MotorControl_GoStraight3();
        // AML_MotorControl_GoStraight4();

        // AML_MotoControl_GoStraight5();

        AML_MotorControl_LeftMotorSpeed(300);
    }
    else if (htim->Instance == htim10.Instance) // timer for led control
    {
        // AML_DebugDevice_Handle();
        AML_LaserSensor_ReadAll();
        // debug[30]++;
    }
    // debug[11]++;
}

void AML_MotorControl_TurnOnWallFollow(void)
{
    HAL_TIM_Base_Start_IT(&htim9);
}

void AML_MotorControl_TurnOffWallFollow(void)
{
    HAL_TIM_Base_Stop_IT(&htim9);
}

void AML_MotorControl_MoveForward_mm(uint16_t distance)
{
    AML_Encoder_ResetLeftValue();
    // AML_Encoder_ResetRightValue();
    AML_MotorControl_TurnOnWallFollow();

    uint16_t ticks = (uint16_t)((double)distance / (Pi * WheelDiameter) * PulsePerRound * Ratio);

    // ticks = 330;

    while (AML_Encoder_GetLeftValue() < ticks)
    {
    }

    AML_MotorControl_TurnOffWallFollow();

    // AML_MotorControl_ShortBreak('F');

    // AML_MotorControl_Stop();

    AML_Encoder_ResetLeftValue();
}

void AML_MotorControl_AdvanceTicks(int16_t ticks)
{
    AML_Encoder_ResetLeftValue();
    AML_MPUSensor_ResetAngle();

    AML_MotorControl_TurnOnWallFollow();

    while (ReadButton != 2 && AML_Encoder_GetLeftValue() < ticks && AML_LaserSensor_ReadSingleWithoutFillter(FF) > 55)
    {
    }

    AML_MotorControl_TurnOffWallFollow();

    AML_Encoder_ResetLeftValue();
}

void AML_MotorControl_TurnLeft90(void)
{
    AML_MotorControl_ShortBreak('F');

    if (RemarkAfterTurnMode == 1)
    {
        uint8_t CalibFlag = AML_LaserSensor_ReadSingleWithoutFillter(BR) < (WALL_IN_RIGHT + 20);
        uint32_t delay;

        if (CalibFlag == 1)
        {
            delay = 1000;
        }
        else
        {
            delay = 2500;
        }

        AML_DebugDevice_BuzzerBeep(20);
        AML_PID_TurnLeft.Setpoint = TempSetpoint + TurnLeftAngle;

        HAL_Delay(200);

        uint32_t InitTime = HAL_GetTick();
        uint32_t CurrentTime = HAL_GetTick();
        uint32_t PreviousTime = CurrentTime;

        while ((CurrentTime - PreviousTime) < 150 && (HAL_GetTick() - InitTime < delay))
        {
            AML_PID_TurnLeft.Input = AML_MPUSensor_GetAngle();

            AML_PID_Compute(&AML_PID_TurnLeft);

            AML_MotorControl_LeftPWM(-(int32_t)AML_PID_TurnLeft.Output);
            AML_MotorControl_RightPWM((int32_t)AML_PID_TurnLeft.Output);

            if (ABS(AML_PID_TurnLeft.Input - AML_PID_TurnLeft.Setpoint) < 3.0f)
            {
                CurrentTime = HAL_GetTick();
            }
            else
            {
                CurrentTime = HAL_GetTick();
                PreviousTime = CurrentTime;
            }
        }

        AML_MotorControl_Stop();

        if (ABS(AML_PID_TurnLeft.Input - AML_PID_TurnLeft.Setpoint) > 35.0f)
        {
            HAL_Delay(60000);
        }

        if (CalibFlag == 1)
        {
            AML_MotorControl_LeftPWM(-20);
            AML_MotorControl_RightPWM(-20);
            HAL_Delay(550);
            AML_MPUSensor_ResetAngle();
            HAL_Delay(70);
            AML_MotorControl_Stop();
            *PID_MPUFollow.MyOutput = 0;
            TempSetpoint = 0;
            HAL_Delay(50);
        }
        else
        {
            TempSetpoint += TurnLeftAngle;
        }
    }
    else if (RemarkAfterTurnMode == 0)
    {
        AML_PID_TurnLeft.Setpoint = TempSetpoint + TurnLeftAngle;

        // AML_PID_TurnLeft.Setpoint = TurnLeftAngle - AML_MPUSensor_GetAngle();

        AML_MPUSensor_ResetAngle();

        // HAL_Delay(150);

        AML_DebugDevice_BuzzerBeep(50);
        HAL_Delay(50);
        AML_DebugDevice_BuzzerBeep(50);

        uint32_t InitTime = HAL_GetTick();
        uint32_t CurrentTime = HAL_GetTick();
        uint32_t PreviousTime = CurrentTime;

        while ((CurrentTime - PreviousTime) < 200 && (HAL_GetTick() - InitTime < 1000))
        {
            AML_PID_TurnLeft.Input = AML_MPUSensor_GetAngle();

            AML_PID_Compute(&AML_PID_TurnLeft);

            AML_MotorControl_LeftPWM(-(int32_t)AML_PID_TurnLeft.Output);
            AML_MotorControl_RightPWM((int32_t)AML_PID_TurnLeft.Output);

            if (ABS(AML_PID_TurnLeft.Input - AML_PID_TurnLeft.Setpoint) < 2.0f)
            {
                CurrentTime = HAL_GetTick();
            }
            else
            {
                CurrentTime = HAL_GetTick();
                PreviousTime = CurrentTime;
            }
        }

        AML_MotorControl_Stop();

        if (ABS(AML_PID_TurnLeft.Input - AML_PID_TurnLeft.Setpoint) > 35.0f)
        {
            HAL_Delay(60000);
        }

        TempSetpoint = AML_PID_TurnLeft.Setpoint - AML_MPUSensor_GetAngle();
        // TempSetpoint += TurnLeftAngle;
        AML_MPUSensor_ResetAngle();
        // TempSetpoint = 0;
    }
}

void AML_MotorControl_TurnRight90(void)
{
    AML_MotorControl_ShortBreak('F');

    if (RemarkAfterTurnMode == 1)
    {
        uint8_t CalibFlag = AML_LaserSensor_ReadSingleWithoutFillter(BL) < (WALL_IN_LEFT + 20);
        uint32_t delay;

        if (CalibFlag)
        {
            delay = 1000;
        }
        else
        {
            delay = 2500;
        }

        AML_DebugDevice_BuzzerBeep(20);
        AML_PID_TurnRight.Setpoint = TempSetpoint + TurnRightAngle;

        HAL_Delay(150);

        uint32_t InitTime = HAL_GetTick();
        uint32_t CurrentTime = HAL_GetTick();
        uint32_t PreviousTime = CurrentTime;

        while ((CurrentTime - PreviousTime) < 200 && (HAL_GetTick() - InitTime) < delay)
        {
            AML_PID_TurnRight.Input = AML_MPUSensor_GetAngle();

            AML_PID_Compute(&AML_PID_TurnRight);

            AML_MotorControl_LeftPWM(-(int32_t)AML_PID_TurnRight.Output);
            AML_MotorControl_RightPWM((int32_t)AML_PID_TurnRight.Output);

            if (ABS(AML_PID_TurnRight.Input - AML_PID_TurnRight.Setpoint) < 2.0f)
            {
                CurrentTime = HAL_GetTick();
            }
            else
            {
                CurrentTime = HAL_GetTick();
                PreviousTime = CurrentTime;
            }
        }

        // AML_MotorControl_ShortBreak('R');
        AML_MotorControl_Stop();

        if (ABS(AML_PID_TurnRight.Input - AML_PID_TurnRight.Setpoint) > 35.0f)
        {
            HAL_Delay(60000);
        }

        // HAL_Delay(150);

        if (CalibFlag == 1)
        {
            AML_MotorControl_LeftPWM(-20);
            AML_MotorControl_RightPWM(-20);
            HAL_Delay(550);
            AML_MPUSensor_ResetAngle();
            HAL_Delay(70);
            AML_MotorControl_Stop();
            *PID_MPUFollow.MyOutput = 0;
            TempSetpoint = 0;
            HAL_Delay(50);
        }
        else
        {
            TempSetpoint += TurnRightAngle;
        }

        // *PID_TurnLeft.MyOutput = 0;
        // *PID_TurnRight.MyOutput = 0;
    }
    else if (RemarkAfterTurnMode == 0)
    {
        AML_PID_TurnRight.Setpoint = TempSetpoint + TurnRightAngle;
        // AML_PID_TurnRight.Setpoint = TurnRightAngle - AML_MPUSensor_GetAngle();

        AML_MPUSensor_ResetAngle();

        // HAL_Delay(150);

        AML_DebugDevice_BuzzerBeep(50);
        HAL_Delay(50);
        AML_DebugDevice_BuzzerBeep(50);

        uint32_t InitTime = HAL_GetTick();
        uint32_t CurrentTime = HAL_GetTick();
        uint32_t PreviousTime = CurrentTime;

        while ((CurrentTime - PreviousTime) < 200 && (HAL_GetTick() - InitTime < 1000))
        {
            AML_PID_TurnRight.Input = AML_MPUSensor_GetAngle();
            // PID_Compute(&PID_TurnRight);

            AML_PID_Compute(&AML_PID_TurnRight);

            AML_MotorControl_LeftPWM(-(int32_t)AML_PID_TurnRight.Output);
            AML_MotorControl_RightPWM((int32_t)AML_PID_TurnRight.Output);

            if (ABS(AML_PID_TurnRight.Input - AML_PID_TurnRight.Setpoint) < 2.0f)
            {
                CurrentTime = HAL_GetTick();
            }
            else
            {
                CurrentTime = HAL_GetTick();
                PreviousTime = CurrentTime;
            }
        }

        // TempSetpoint += TurnRightAngle;

        AML_MotorControl_Stop();

        if (ABS(AML_PID_TurnRight.Input - AML_PID_TurnRight.Setpoint) > 35.0f)
        {
            HAL_Delay(60000);
        }

        TempSetpoint = AML_PID_TurnRight.Setpoint - AML_MPUSensor_GetAngle();
        AML_MPUSensor_ResetAngle();
        // TempSetpoint = 0;
    }
}

void AML_MotorControl_TurnLeft180(void)
{
    AML_MotorControl_ShortBreak('F');

    if (RemarkAfterTurnMode == 1)
    {
        // double TempAngle = AML_MPUSensor_GetAngle();

        // AML_MPUSensor_ResetAngle();
        AML_DebugDevice_BuzzerBeep(50);
        HAL_Delay(50);
        AML_DebugDevice_BuzzerBeep(50);

        AML_PID_TurnLeft.Setpoint = TempSetpoint + BackwardLeftAngle;

        // AML_MotorControl_TurnOffWallFollow();

        HAL_Delay(150);

        uint32_t InitTime = HAL_GetTick();
        uint32_t CurrentTime = HAL_GetTick();
        uint32_t PreviousTime = CurrentTime;

        while ((CurrentTime - PreviousTime) < 200 && (HAL_GetTick() - InitTime < 2500))
        {
            AML_PID_TurnLeft.Input = AML_MPUSensor_GetAngle();

            AML_PID_Compute(&AML_PID_TurnLeft);

            AML_MotorControl_LeftPWM(-(int32_t)AML_PID_TurnLeft.Output);

            AML_MotorControl_RightPWM((int32_t)AML_PID_TurnLeft.Output);

            if (ABS(AML_PID_TurnLeft.Input - AML_PID_TurnLeft.Setpoint) < 2.0f)
            {
                CurrentTime = HAL_GetTick();
            }
            else
            {
                CurrentTime = HAL_GetTick();
                PreviousTime = CurrentTime;
            }
        }

        // AML_MotorControl_ShortBreak('L');
        // HAL_Delay(200);

        AML_MPUSensor_ResetAngle();
        *PID_TurnLeft.MyOutput = 0;
        *PID_TurnRight.MyOutput = 0;
        *PID_MPUFollow.MyOutput = 0;

        // AML_MotorControl_PIDReset(&PID_MPUFollow);
        TempSetpoint = 0;
    }
    else if (RemarkAfterTurnMode == 0)
    {
        // AML_PID_TurnLeft.Setpoint = TempSetpoint + 175.0f;

        AML_PID_TurnLeft.Setpoint = BackwardLeftAngle - AML_MPUSensor_GetAngle();

        AML_MPUSensor_ResetAngle();

        // HAL_Delay(150);

        AML_DebugDevice_BuzzerBeep(50);
        HAL_Delay(50);
        AML_DebugDevice_BuzzerBeep(50);

        uint32_t InitTime = HAL_GetTick();
        uint32_t CurrentTime = HAL_GetTick();
        uint32_t PreviousTime = CurrentTime;

        while ((CurrentTime - PreviousTime) < 200 && (HAL_GetTick() - InitTime < 1300))
        {
            AML_PID_TurnLeft.Input = AML_MPUSensor_GetAngle();
            // PID_Compute(&PID_TurnLeft);

            AML_PID_Compute(&AML_PID_TurnLeft);

            AML_MotorControl_LeftPWM(-(int32_t)AML_PID_TurnLeft.Output);

            AML_MotorControl_RightPWM((int32_t)AML_PID_TurnLeft.Output);

            if (ABS(AML_PID_TurnLeft.Input - AML_PID_TurnLeft.Setpoint) < 2.0f)
            {
                CurrentTime = HAL_GetTick();
            }
            else
            {
                CurrentTime = HAL_GetTick();
                PreviousTime = CurrentTime;
            }
        }

        // TempSetpoint += TurnLeftAngle;

        AML_MPUSensor_ResetAngle();
        TempSetpoint = 0;
    }

    AML_DebugDevice_TurnOffLED(5);
}

void AML_MotorControl_TurnRight180(void)
{
    AML_MotorControl_ShortBreak('F');

    if (RemarkAfterTurnMode == 1)
    {
        // double TempAngle = AML_MPUSensor_GetAngle();

        // AML_MPUSensor_ResetAngle();
        AML_DebugDevice_BuzzerBeep(20);

        AML_PID_TurnRight.Setpoint = TempSetpoint + BackwardRightAngle;

        AML_MotorControl_TurnOffWallFollow();

        HAL_Delay(150);

        uint32_t InitTime = HAL_GetTick();
        uint32_t CurrentTime = HAL_GetTick();
        uint32_t PreviousTime = CurrentTime;

        while ((CurrentTime - PreviousTime) < 200 && (HAL_GetTick() - InitTime < 1300))
        {
            AML_PID_TurnRight.Input = AML_MPUSensor_GetAngle();

            AML_PID_Compute(&AML_PID_TurnRight);

            AML_MotorControl_LeftPWM(-(int32_t)AML_PID_TurnRight.Output);
            AML_MotorControl_RightPWM((int32_t)AML_PID_TurnRight.Output);

            if (ABS(AML_PID_TurnRight.Input - AML_PID_TurnRight.Setpoint) < 2.0f)
            {
                CurrentTime = HAL_GetTick();
            }
            else
            {
                CurrentTime = HAL_GetTick();
                PreviousTime = CurrentTime;
            }
        }

        // AML_MotorControl_ShortBreak('R');
        // HAL_Delay(200);

        AML_MPUSensor_ResetAngle();
        *PID_TurnLeft.MyOutput = 0;
        *PID_TurnRight.MyOutput = 0;
        *PID_MPUFollow.MyOutput = 0;

        // AML_MotorControl_PIDReset(&PID_MPUFollow);
        TempSetpoint = 0;
    }
    else if (RemarkAfterTurnMode == 0)
    {
        // AML_PID_TurnRight.Setpoint = TempSetpoint + 175.0f;

        AML_PID_TurnRight.Setpoint = BackwardRightAngle - AML_MPUSensor_GetAngle();

        AML_MPUSensor_ResetAngle();

        // HAL_Delay(150);

        AML_DebugDevice_BuzzerBeep(50);
        HAL_Delay(50);
        AML_DebugDevice_BuzzerBeep(50);

        uint32_t InitTime = HAL_GetTick();
        uint32_t CurrentTime = HAL_GetTick();
        uint32_t PreviousTime = CurrentTime;

        while ((CurrentTime - PreviousTime) < 200 && (HAL_GetTick() - InitTime < 1300))
        {
            AML_PID_TurnRight.Input = AML_MPUSensor_GetAngle();
            // PID_Compute(&PID_TurnRight);

            AML_PID_Compute(&AML_PID_TurnRight);

            AML_MotorControl_LeftPWM(-(int32_t)AML_PID_TurnRight.Output);
            AML_MotorControl_RightPWM((int32_t)AML_PID_TurnRight.Output);

            if (ABS(AML_PID_TurnRight.Input - AML_PID_TurnRight.Setpoint) < 2.0f)
            {
                CurrentTime = HAL_GetTick();
            }
            else
            {
                CurrentTime = HAL_GetTick();
                PreviousTime = CurrentTime;
            }
        }

        // TempSetpoint += TurnRightAngle;
        TempSetpoint = 0;
        AML_MPUSensor_ResetAngle();
    }
}

void AML_MotorControl_LeftStillTurn(void)
{

    AML_DebugDevice_BuzzerBeep(20);

    AML_MotorControl_AdvanceTicks((int16_t)(FLOOD_ENCODER_TICKS_ONE_CELL / 2));
    AML_MotorControl_ShortBreak('F');

    AML_MotorControl_TurnLeft90();

    AML_MotorControl_AdvanceTicks((int16_t)(FLOOD_ENCODER_TICKS_ONE_CELL / 2));

    AML_MotorControl_TurnOnWallFollow();
}

void AML_MotorControl_RightStillTurn(void)
{
    AML_DebugDevice_BuzzerBeep(20);

    AML_MotorControl_AdvanceTicks((int16_t)(FLOOD_ENCODER_TICKS_ONE_CELL / 2));
    AML_MotorControl_ShortBreak('F');

    AML_MotorControl_TurnRight90();

    AML_MotorControl_AdvanceTicks((int16_t)(FLOOD_ENCODER_TICKS_ONE_CELL / 2));

    AML_MotorControl_TurnOnWallFollow();
}

void AML_MotorControl_BackStillTurn(void)
{
    AML_DebugDevice_BuzzerBeep(20);

    AML_MotorControl_TurnOffWallFollow();
    AML_MotorControl_ShortBreak('F');

    AML_MotorControl_TurnLeft180();

    AML_MotorControl_TurnOnWallFollow();
}

/////////////////////////////////////////////////////////////

void AML_MotroControl_FloodLeftStillTurn(void)
{
    AML_DebugDevice_BuzzerBeep(20);

    AML_MotorControl_AdvanceTicks((int16_t)(FLOOD_ENCODER_TICKS_ONE_CELL / 2));
    AML_MotorControl_ShortBreak('F');

    AML_MotorControl_TurnLeft90();

    AML_MotorControl_AdvanceTicks((int16_t)(FLOOD_ENCODER_TICKS_ONE_CELL / 2));

    AML_MotorControl_TurnOnWallFollow();
}

void AML_MotorControl_LeftMotorSpeed(int32_t rpm)
{
    PID_LeftMotor.Setpoint = (double)rpm;

    PID_LeftMotor.Input = ((double)AML_Encoder_GetLeftValue() / 192) * (1000 / PID_LeftMotor.sampleTime) * 60; // 50 is the sample time
    AML_Encoder_ResetLeftValue();

    AML_PID_Compute(&PID_LeftMotor);

    AML_MotorControl_LeftPWM((int32_t)PID_LeftMotor.Output);
}