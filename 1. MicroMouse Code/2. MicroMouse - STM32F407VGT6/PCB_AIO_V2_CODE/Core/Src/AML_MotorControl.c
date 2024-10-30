#include <AML_MotorControl.h>

extern TIM_HandleTypeDef htim1; // timer for pwm
GPIO_TypeDef *MotorDirectionPort = GPIOE;

// TimerClock is 240MHz, Prescaler is 12000, AutoReload is 1, so the frequency is 10kHz
#define TO_CCR(x) (uint16_t)((x) * 100)

// PID struct-------------------------------------------------------------------------------------------------------//
AML_PID_Struct PID_TurnLeft;
double PID_TurnLeft_Kp = 1;
double PID_TurnLeft_Ki = 0;
double PID_TurnLeft_Kd = 0;
double PID_TurnLeft_Tau = 0;
double PID_TurnLeft_Input, PID_TurnLeft_Output, PID_TurnLeft_Setpoint = 0;



// PID setup function-------------------------------------------------------------------------------------------------------//


// Motor control function-------------------------------------------------------------------------------------------------------//

// init motor control
void AML_MotorControl_Setup(void)
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    AML_MotorControl_PIDSetup();
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
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN1_Pin, MotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN2_Pin, !MotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN1_Pin, !MotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN2_Pin, MotorDirection);
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
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, !MotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, MotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, MotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, !MotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}
