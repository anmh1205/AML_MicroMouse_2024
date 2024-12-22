#include "AML_IMU.h"

uint8_t ResetCommand[] = {0xFF, 0xAA, 0x52};

// extern UART_HandleTypeDef huart1;
// extern DMA_HandleTypeDef hdma_usart3_rx;

UART_HandleTypeDef *IMU_UART;

volatile uint8_t MPUData[36];
volatile uint8_t buffer = 119;
// volatile uint8_t index = 0;
// extern int16_t debug[100];
volatile double Angle, PreviousAngle = 0, SaveAngle = 0;
volatile uint8_t error = 0;

//-------------------------------------------------------------------------------------------------------//
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void AML_MPUSensor_HardResetAngle(void);
void AML_MPUSensor_Setup(UART_HandleTypeDef *imu_uart);
void handle(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
double AML_MPUSensor_GetAngle(void);

//-------------------------------------------------------------------------------------------------------//

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);

    if (huart->Instance == USART1)
    {
    }
}

void AML_MPUSensor_HardResetAngle(void)
{
    HAL_UART_DMAStop(IMU_UART);
    HAL_UART_Transmit(IMU_UART, ResetCommand, 3, 1000);
    SaveAngle = 0;
    PreviousAngle = 0;
    Angle = 0;
    HAL_Delay(5);

    HAL_UART_Transmit(IMU_UART, ResetCommand, 3, 1000);
    SaveAngle = 0;
    PreviousAngle = 0;
    Angle = 0;
    HAL_Delay(5);

    HAL_UART_Receive_DMA(IMU_UART, MPUData, 33);
}



void AML_MPUSensor_Setup(UART_HandleTypeDef *imu_uart)
{
    IMU_UART = imu_uart;

    AML_MPUSensor_HardResetAngle();
    HAL_UART_Receive_DMA(IMU_UART, MPUData, 33);
}

void handle(void)
{
    while (buffer != 85) // wait 0x55
    {
        HAL_UART_Receive(IMU_UART, &buffer, 1, 1000);
    }
    buffer = 100;
    HAL_UART_Receive_DMA(IMU_UART, MPUData, 33);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    if (huart->Instance == IMU_UART->Instance)
    {
        if (MPUData[0] != 83)
        {
            handle();
            return;
        }

        PreviousAngle = Angle;
        Angle = (((MPUData[6] << 8) | MPUData[5]) / 32768.0) * 180;

        if (Angle - PreviousAngle > 250.0f) // 0 -> 360 degree
        {
            SaveAngle += 360.0f;
        }
        else if (Angle - PreviousAngle < -250.0f) // 360 -> 0 degree
        {
            SaveAngle -= 360.0f;
        }

        HAL_UART_Receive_DMA(IMU_UART, MPUData, 33);
    }
}

double AML_MPUSensor_GetAngle(void)
{
    return Angle - SaveAngle;
}
