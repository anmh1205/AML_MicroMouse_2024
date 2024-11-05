#include "AML_IRSensor.h"

extern debug[100];

extern ADC_HandleTypeDef hadc2;

uint32_t IRSensorValue[7];
uint8_t ADCIndex = 0;

//-------------------------------------------------------------------------------------------------------//
void AML_IRSensor_Setup(void);

//-------------------------------------------------------------------------------------------------------//

void AML_IRSensor_Setup(void)
{
    memset(IRSensorValue, 0, sizeof(IRSensorValue));

    // HAL_ADC_Start_DMA(&hadc2, IRSensorValue, 7);
    HAL_ADC_Start_IT(&hadc2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    //     UNUSED(hadc);

    //     if (hadc->Instance == ADC2)
    //     {

    //     }
    // HAL_ADC_Stop_IT(&hadc2);

    IRSensorValue[ADCIndex] = HAL_ADC_GetValue(hadc);
    ADCIndex++;

    if (ADCIndex == 7)
    {
        ADCIndex = 0;
        // HAL_ADC_Start_IT(&hadc2);
    }
}

//-------------------------------------------------------------------------------------------------------//

void AML_ReadAll_IRSensor(void)
{
}

void AML_Read_IRSensor(uint8_t sensor)
{
}

uint32_t AML_Get_IRSensor(uint8_t sensor)
{
    return IRSensorValue[sensor];
}

//-------------------------------------------------------------------------------------------------------//