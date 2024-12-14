#ifndef AML_PID_H
#define AML_PID_H

// #include "stm32h7xx.h"
#include "stm32f4xx.h"
#include "main.h"

typedef struct
{
    double Input;
    double Output;
    double Setpoint;

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
} AML_PID_Struct;

// void AML_PID_Init(AML_PID_Struct *pid, double *input, double *output, double *setpoint, double kp, double ki, double kd, double tau, double limMin, double limMax, uint32_t sampleTime);
double AML_PID_Compute(AML_PID_Struct *pid);

#endif // AML_PID_H