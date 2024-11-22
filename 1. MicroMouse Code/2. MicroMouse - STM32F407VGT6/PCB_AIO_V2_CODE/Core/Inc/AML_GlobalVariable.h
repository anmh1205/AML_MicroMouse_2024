#ifndef AML_GLOBAL_H
#define AML_GLOBAL_H

#include "stm32h7xx_hal.h"

#include <AML_MotorControl.h>
#include <AML_IRSensor.h>

// DEFINE FOR PINOUT-------------------------------------------------------------------------------------------------------//

#define SW_0 0
#define SW_1 1

#define BIT_SW_0 0
#define BIT_SW_1 1
#define BIT_SW_2 2
#define BIT_SW_3 3
#define BIT_SW_4 4

#define LED_0 0
#define LED_1 1
#define LED_2 2
#define LED_3 3

#define IR_SENSOR_FF 0
#define IR_SENSOR_FL 1
#define IR_SENSOR_RL 2
#define IR_SENSOR_BL 3
#define IR_SENSOR_BR 4
#define IR_SENSOR_RR 5
#define IR_SENSOR_FR 6


// DEFINE FOR MOTOR CONTROL-------------------------------------------------------------------------------------------------------//
#define MotorDirection 0 // use for change direction of motor
#define LeftMotorDirection MotorDirection // use for change direction of left motor
#define RightMotorDirection !MotorDirection // use for change direction of right motor

// define parameter for transmission function
#define Pi 3.14159265359  // Pi number
#define WheelDiameter 21  // mm
#define TransmissionRatio 1     // ratio between wheel and encoder
#define EncoderPulsePerRound 1420 // 1420 pulse per round encoder
#define MouseSpeed 25 // % of duty cycle

// define parameter for PID control

#define SampleTime 20 // time (ms) for each sample
#define PIDMode 1 // automatic mode
#define PIDOutputMin -MouseSpeed // minimum output of PID (duty cycle)
#define PIDOutputMax MouseSpeed // maximum output of PID (duty cycle)

#define PIDSpeedOutputMin 0 // minimum output of PID (duty cycle)
#define PIDSpeedOutputMax 50 // maximum output of PID (duty cycle)

// DEFINE FOR IR SENSOR-------------------------------------------------------------------------------------------------------//

// competition maze parameters
#define WALL_IN_FRONT 155       // 47
#define WALL_IN_LEFT 55        // 121
#define WALL_IN_RIGHT 55      // 100
#define WALL_IN_FRONT_LEFT 130  // 60
#define WALL_IN_FRONT_RIGHT 130 // 50

#define WALL_NOT_IN_FRONT 170
#define WALL_NOT_IN_LEFT 100
#define WALL_NOT_IN_RIGHT 100
#define WALL_NOT_IN_FRONT_LEFT 155
#define WALL_NOT_IN_FRONT_RIGHT 155
//-------------------------------------------------------------------------------------------------------//

#define CHECK_WALL_FRONT AML_IRSensor_IsFrontWall()
#define CHECK_WALL_LEFT AML_IRSensor_IsLeftWall()
#define CHECK_WALL_RIGHT AML_IRSensor_IsRightWall()

#define MOVE_FORWARD_FUNCTION AML_MotorControl_Stop()
#define TURN_LEFT_FUNCTION AML_MotorControl_Stop()
#define TURN_RIGHT_FUNCTION AML_MotorControl_Stop()


//-------------------------------------------------------------------------------------------------------//

#endif