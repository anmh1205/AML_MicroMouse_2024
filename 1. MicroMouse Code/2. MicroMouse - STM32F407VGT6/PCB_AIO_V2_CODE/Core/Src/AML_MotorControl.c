#include <AML_MotorControl.h>

extern debug[100];

extern TIM_HandleTypeDef htim1; // timer for pwm
extern TIM_HandleTypeDef htim7; // timer for interrupt

GPIO_TypeDef *MotorDirectionPort = GPIOE;

// TimerClock is 240MHz, Prescaler is 12000, AutoReload is 1, so the frequency is 10kHz
#define TO_CCR(x) (uint16_t)((x) * 10)

// PID struct-------------------------------------------------------------------------------------------------------//

double TempSetpoint = 0;

AML_PID_Struct PID_LeftMotor;
double PID_LeftMotor_Kp = 8;
double PID_LeftMotor_Ki = 1;
double PID_LeftMotor_Kd = 2;
double PID_LeftMotor_Tau = 1;
double PID_LeftMotor_Input, PID_LeftMotor_Output, PID_LeftMotor_Setpoint = 0;

AML_PID_Struct PID_RightMotor;
double PID_RightMotor_Kp = 1;
double PID_RightMotor_Ki = 0;
double PID_RightMotor_Kd = 0;
double PID_RightMotor_Tau = 0;
double PID_RightMotor_Input, PID_RightMotor_Output, PID_RightMotor_Setpoint = 0;


AML_PID_Struct PID_TurnLeft;
double PID_TurnLeft_Kp = 1;
double PID_TurnLeft_Ki = 0;
double PID_TurnLeft_Kd = 0;
double PID_TurnLeft_Tau = 0;
double PID_TurnLeft_Input, PID_TurnLeft_Output, PID_TurnLeft_Setpoint = 0;

AML_PID_Struct PID_TurnRight;
double PID_TurnRight_Kp = 1;
double PID_TurnRight_Ki = 0;
double PID_TurnRight_Kd = 0;
double PID_TurnRight_Tau = 0;
double PID_TurnRight_Input, PID_TurnRight_Output, PID_TurnRight_Setpoint = 0;

AML_PID_Struct PID_MPUFollow;
double PID_MPUFollow_Kp = 0.5;
double PID_MPUFollow_Ki = 0.1;
double PID_MPUFollow_Kd = 0;
double PID_MPUFollow_Tau = 0;
double PID_MPUFollow_Input, PID_MPUFollow_Output, PID_MPUFollow_Setpoint = 0;

AML_PID_Struct PID_LeftWallFollow;
double PID_LeftWallFollow_Kp = 1;
double PID_LeftWallFollow_Ki = 0;
double PID_LeftWallFollow_Kd = 0;
double PID_LeftWallFollow_Tau = 0;
double PID_LeftWallFollow_Input, PID_LeftWallFollow_Output, PID_LeftWallFollow_Setpoint = 0;

AML_PID_Struct PID_RightWallFollow;
double PID_RightWallFollow_Kp = 1;
double PID_RightWallFollow_Ki = 0;
double PID_RightWallFollow_Kd = 0;
double PID_RightWallFollow_Tau = 0;
double PID_RightWallFollow_Input, PID_RightWallFollow_Output, PID_RightWallFollow_Setpoint = 0;

// FUNCTION-----------------------------------------------------------------------------------------------------------------//
void AML_MotorControl_AMLPIDSetup(void);
void AML_MotorControl_Setup(void);
void AML_MotorControl_LeftPWM(int32_t DutyCycle);
void AML_MotorControl_RightPWM(int32_t DutyCycle);
void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle);
void AML_MotorControl_Stop(void);

void AML_MotorControl_GoStraghtWithMPU(double setpoint);
void AML_MotorControl_LeftWallFollow(void);
void AML_MotorControl_RightWallFollow(void);
void AML_MotorControl_GoStraight(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
    if (htim->Instance == htim7.Instance) // timer for wall follow
    {
        AML_MotorControl_GoStraghtWithMPU(0);

        // AML_MotorControl_LeftMotorSpeed(10);
    }
}

void AML_MotorControl_TurnOnWallFollow(void)
{
    AML_MPUSensor_ResetAngle();
    HAL_TIM_Base_Start_IT(&htim7);
}

void AML_MotorControl_TurnOffWallFollow(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
}

// PID setup function-------------------------------------------------------------------------------------------------------//
void AML_MotorControl_AMLPIDSetup(void)
{
    AML_PID_Init(&PID_LeftMotor, &PID_LeftMotor_Input, &PID_LeftMotor_Output, &PID_LeftMotor_Setpoint, PID_LeftMotor_Kp, PID_LeftMotor_Ki, PID_LeftMotor_Kd, PID_LeftMotor_Tau, PIDSpeedOutputMin, PIDSpeedOutputMax, SampleTime);
    AML_PID_Init(&PID_RightMotor, &PID_RightMotor_Input, &PID_RightMotor_Output, &PID_RightMotor_Setpoint, PID_RightMotor_Kp, PID_RightMotor_Ki, PID_RightMotor_Kd, PID_RightMotor_Tau, PIDSpeedOutputMin, PIDSpeedOutputMax, SampleTime);

    AML_PID_Init(&PID_TurnLeft, &PID_TurnLeft_Input, &PID_TurnLeft_Output, &PID_TurnLeft_Setpoint, PID_TurnLeft_Kp, PID_TurnLeft_Ki, PID_TurnLeft_Kd, PID_TurnLeft_Tau, PIDOutputMin, PIDOutputMax, SampleTime);
    AML_PID_Init(&PID_TurnRight, &PID_TurnRight_Input, &PID_TurnRight_Output, &PID_TurnRight_Setpoint, PID_TurnRight_Kp, PID_TurnRight_Ki, PID_TurnRight_Kd, PID_TurnRight_Tau, PIDOutputMin, PIDOutputMax, SampleTime);

    AML_PID_Init(&PID_MPUFollow, &PID_MPUFollow_Input, &PID_MPUFollow_Output, &PID_MPUFollow_Setpoint, PID_MPUFollow_Kp, PID_MPUFollow_Ki, PID_MPUFollow_Kd, PID_MPUFollow_Tau, PIDOutputMin, PIDOutputMax, SampleTime);
    AML_PID_Init(&PID_LeftWallFollow, &PID_LeftWallFollow_Input, &PID_LeftWallFollow_Output, &PID_LeftWallFollow_Setpoint, PID_LeftWallFollow_Kp, PID_LeftWallFollow_Ki, PID_LeftWallFollow_Kd, PID_LeftWallFollow_Tau, PIDOutputMin, PIDOutputMax, SampleTime);
    AML_PID_Init(&PID_RightWallFollow, &PID_RightWallFollow_Input, &PID_RightWallFollow_Output, &PID_RightWallFollow_Setpoint, PID_RightWallFollow_Kp, PID_RightWallFollow_Ki, PID_RightWallFollow_Kd, PID_RightWallFollow_Tau, PIDOutputMin, PIDOutputMax, SampleTime);
}

// Motor control function-------------------------------------------------------------------------------------------------------//

// init motor control
void AML_MotorControl_Setup(void)
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    AML_MotorControl_AMLPIDSetup();
}


void AML_MotorControl_LeftPWM(int32_t DutyCycle)
{
    if (DutyCycle > 100)
    {
        DutyCycle = 100;
    }
    else if (DutyCycle < -100)
    {
        DutyCycle = -100;
    }

    if (DutyCycle > 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN1_Pin, LeftMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN2_Pin, !LeftMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN1_Pin, !LeftMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN2_Pin, LeftMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}

void AML_MotorControl_RightPWM(int32_t DutyCycle)
{
    DutyCycle = (int32_t)(DutyCycle * 1);

    if (DutyCycle > 100)
    {
        DutyCycle = 100;
    }
    else if (DutyCycle < -100)
    {
        DutyCycle = -100;
    }

    if (DutyCycle > 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, RightMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, !RightMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, !RightMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, RightMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle)
{
    AML_MotorControl_LeftPWM(LeftDutyCycle);
    AML_MotorControl_RightPWM(RightDutyCycle);
}

void AML_MotorControl_Stop(void)
{
    AML_MotorControl_Move(0, 0);
}

void AML_MotorControl_LeftMotorSpeed(int32_t rps)
{
    PID_LeftMotor_Setpoint = (double)rps;

    PID_LeftMotor_Input = ((double)AML_Encoder_GetLeftValue() / EncoderPulsePerRound) * 50 ; // 50 is the sample time
    AML_Encoder_ResetLeftValue();

    AML_PID_Compute(&PID_LeftMotor);

    AML_MotorControl_LeftPWM((int32_t)PID_LeftMotor_Output);
}

void AML_MotorControl_RightMotorSpeed(int32_t rps)
{
    PID_RightMotor_Setpoint = (double)rps;

    PID_RightMotor_Input = (double) AML_Encoder_GetRightValue() / EncoderPulsePerRound * 50; // 50 is the sample time
    AML_Encoder_ResetRightValue();

    AML_PID_Compute(&PID_RightMotor);

    AML_MotorControl_RightPWM((int32_t)PID_RightMotor_Output);
}

//--------------------------------------------------------------------------------------------------------//

void AML_MotorControl_GoStraghtWithMPU(double setpoint)
{
    PID_MPUFollow_Input = AML_MPUSensor_GetAngle();
    PID_MPUFollow_Setpoint = setpoint;

    AML_PID_Compute(&PID_MPUFollow);

    AML_MotorControl_Move(MouseSpeed - (int32_t)PID_MPUFollow_Output, MouseSpeed + (int32_t)PID_MPUFollow_Output);
}

void AML_MotorControl_LeftWallFollow(void)
{
    PID_LeftWallFollow_Input = AML_IRSensor_GetDistance(IR_SENSOR_RL);
    PID_LeftWallFollow_Setpoint = WALL_IN_LEFT;

    AML_PID_Compute(&PID_LeftWallFollow);
}

void AML_MotorControl_RightWallFollow(void)
{
    PID_RightWallFollow_Input = AML_IRSensor_GetDistance(IR_SENSOR_RR);
    PID_RightWallFollow_Setpoint = WALL_IN_RIGHT;

    AML_PID_Compute(&PID_RightWallFollow);
}

void AML_MotorControl_GoStraight(void)
{
    if (AML_IRSensor_GetDistance(IR_SENSOR_RL) > WALL_NOT_IN_LEFT && AML_IRSensor_GetDistance(IR_SENSOR_RR) > WALL_NOT_IN_RIGHT)
    {
        AML_MotorControl_GoStraghtWithMPU(TempSetpoint);
    }
    else if (AML_IRSensor_GetDistance(IR_SENSOR_RL) < WALL_IN_LEFT)
    {
        AML_MotorControl_LeftWallFollow();

        // TempSetpoint = -*PID_LeftWallFollow.MyOutput;

        AML_MotorControl_GoStraghtWithMPU(TempSetpoint - *PID_LeftWallFollow.MyOutput);
    }
    else if (AML_IRSensor_GetDistance(IR_SENSOR_RR) < WALL_IN_RIGHT)
    {
        AML_MotorControl_RightWallFollow();

        // TempSetpoint = *PID_RightWallFollow.MyOutput;

        AML_MotorControl_GoStraghtWithMPU(TempSetpoint + *PID_RightWallFollow.MyOutput);
    }
}

//--------------------------------------------------------------------------------------------------------//

void AML_MotorControl_TurnLeft(void)
{
    uint16_t WaitingTime = 1000;

    PID_TurnLeft_Setpoint = 90;

    uint32_t InitTime = HAL_GetTick();
    uint32_t CurrentTime = HAL_GetTick();
    uint32_t PreviousTime = CurrentTime;

    while ((CurrentTime - PreviousTime) < 200 && (HAL_GetTick() - InitTime < WaitingTime))
    {
        PID_TurnLeft_Input = AML_MPUSensor_GetAngle();

        AML_PID_Compute(&PID_TurnLeft);

        AML_MotorControl_LeftPWM(-(int32_t)PID_TurnLeft_Output);
        AML_MotorControl_RightPWM((int32_t)PID_TurnLeft_Output);

        if (ABS(PID_TurnLeft_Input - PID_TurnLeft_Setpoint) < 2.0f)
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
}
