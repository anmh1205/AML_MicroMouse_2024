#ifndef AML_PID_H
#define AML_PID_H

#include "stm32h7xx.h"

typedef struct
{
    double Kp;
    double Ki;
    double Kd;

    double tau;

    double limMin;
    double limMax;

    double linMinInt;
    double linMaxInt;

    uint32_t sampleTime;
    uint32_t lastTime;

    double integratol;
    double prevError;
    double differentiator;
    double prevMeasurement;

    double out;
} AML_PID_Struct;

void AML_PID_Init(AML_PID_Struct *pid, double kp, double ki, double kd, double tau, double sampleTime);
double AML_PID_Compute(AML_PID_Struct *pid, double measurement, double setpoint);

#endif // AML_PID_H