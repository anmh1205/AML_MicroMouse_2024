#include "AML_IRSensor.h"

/*
    distance low range: y = 33,9 + -69,5x + 62,3x^2 + -25,4x^3 + 3,83x^4
    distance high range: y = 55.38 - 77.99x + 54.73x^2 - 18.33x^3 + 2.29x^4
*/

typedef enum IRSensor
{
    IR_SENSOR_FF,
    IR_SENSOR_FL,
    IR_SENSOR_RL,
    IR_SENSOR_BL,
    IR_SENSOR_BR,
    IR_SENSOR_RR,
    IR_SENSOR_FR
} IRSensor_t;

#define ADC_RESOLUTION 65536
#define ADC_MAX 65535
#define ADC_VREF 3.3

#define GET_VOLTAGE(adcValue) ((adcValue * ADC_VREF) / ADC_MAX)
#define GET_DISTANCE_2_15(voltage) (33.9 + (-69.5 * voltage) + (62.3 * pow(voltage, 2)) + (-25.4 * pow(voltage, 3)) + (3.83 * pow(voltage, 4))) * 10 // mm
#define GET_DISTANCE_4_30(voltage)  (55.38 - (77.99 * voltage) + (54.73 * pow(voltage, 2)) - (18.33 * pow(voltage, 3)) + (2.29 * pow(voltage, 4))) * 10 // mm

#define GET_DISTANCE(voltage, index) (index > 0) ? GET_DISTANCE_2_15(voltage) : GET_DISTANCE_4_30(voltage)

extern debug[100];

extern ADC_HandleTypeDef hadc2;

uint32_t IRSensorADCValue[7];
double IRSensorVoltageValue[7];
double IRSensorDistanceValue[7];

uint8_t ADCIndex = 0;

//-------------------------------------------------------------------------------------------------------//
void AML_IRSensor_Setup(void);

//-------------------------------------------------------------------------------------------------------//

void AML_IRSensor_Setup(void)
{
    memset(IRSensorADCValue, 0, sizeof(IRSensorADCValue));

    // HAL_ADC_Start_DMA(&hadc2, IRSensorValue, 7);
    HAL_ADC_Start_IT(&hadc2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
        UNUSED(hadc);

        if (hadc->Instance == ADC2)
        {
            IRSensorADCValue[ADCIndex] = HAL_ADC_GetValue(hadc);
            IRSensorVoltageValue[ADCIndex] = GET_VOLTAGE(IRSensorADCValue[ADCIndex]);
            IRSensorDistanceValue[ADCIndex] = GET_DISTANCE(IRSensorVoltageValue[ADCIndex], ADCIndex);

            ADCIndex++;

            if (ADCIndex == 7)
            {
                ADCIndex = 0;
            }
        }
}

//-------------------------------------------------------------------------------------------------------//

double AML_IRSensor_GetDistance(uint8_t sensor)
{

}

//-------------------------------------------------------------------------------------------------------//