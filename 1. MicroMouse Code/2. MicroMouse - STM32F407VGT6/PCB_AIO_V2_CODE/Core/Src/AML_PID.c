#include "AML_PID.h"



void AML_PID_Init(AML_PID_Struct *pid, double kp, double ki, double kd, double tau, double sampleTime)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->tau = tau;
    pid->sampleTime = sampleTime;


    pid->integratol = 0;
    pid->prevError = 0;

    pid->differentiator = 0;
    pid->prevMeasurement = 0;

    pid->out = 0;
}

double AML_PID_Compute(AML_PID_Struct *pid, double measurement, double setpoint)
{
    uint32_t now = HAL_GetTick();
    uint32_t timeChange = (now - pid->lastTime);

    if (timeChange >= pid->sampleTime)
    {
        // Compute PID output value for given reference input and feedback
        // pid->out = AML_PID_Compute(pid, setpoint, measurement);

        double error = setpoint - measurement;

        double pTerm = pid->Kp * error;

        pid->integratol += error * pid->sampleTime;

        pid->integratol += 0.5f * pid->Ki * pid->sampleTime * (error + pid->prevError);

        if (pid->integratol > pid->linMaxInt)
        {
            pid->integratol = pid->linMaxInt;
        }
        else if (pid->integratol < pid->linMinInt)
        {
            pid->integratol = pid->linMinInt;
        }

        double iTerm = pid->Ki * pid->integratol;

        pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) + (2.0f * pid->tau - pid->sampleTime) * pid->differentiator) / (2.0f * pid->tau + pid->sampleTime);
        
        double dTerm = pid->Kd * pid->differentiator;


        // pid->out = pTerm + pid->integratol + pid->differentiator;
        pid->out = pTerm + iTerm + dTerm;

        if (pid->out > pid->limMax)
        {
            pid->out = pid->limMax;
        }
        else if (pid->out < pid->limMin)
        {
            pid->out = pid->limMin;
        }

        pid->prevMeasurement = measurement;
        pid->prevError = error;

        // Remember last time for next calculation
        pid->lastTime = now;
    }

    return pid->out;
}