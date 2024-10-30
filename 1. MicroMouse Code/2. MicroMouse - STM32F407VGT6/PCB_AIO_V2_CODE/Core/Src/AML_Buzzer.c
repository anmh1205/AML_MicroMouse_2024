#include "AML_Buzzer.h"

// Bộ định thời cho PWM
extern TIM_HandleTypeDef htim3;

uint16_t ClockAfterPrescal = 60000;

// Tần số của các nốt nhạc cho đoạn "Glory Glory Man United"
const float frequencies[] = {
    392.00, // Sol (G4)
    440.00, // La (A4)
    493.88, // Si (B4)
    523.25, // Đô (C5)
    493.88, // Si (B4)
    440.00, // La (A4)
    392.00, // Sol (G4)
    392.00, // Sol (G4)
    440.00, // La (A4)
    493.88, // Si (B4)
    523.25, // Đô (C5)
    587.33, // Rê (D5)
    659.25, // Mi (E5)
    587.33, // Rê (D5)
    523.25, // Đô (C5)
    493.88, // Si (B4)
    440.00  // La (A4)
};

// Thời gian phát từng nốt (tính theo ms) - dựa trên sheet nhạc
const uint16_t durations[] = {
    500, 500, 500, 500, // Nốt đen
    250, 250, 250, 250, // Nốt móc đơn
    500, 500, 500, 500, // Nốt đen
    750, 500, 500, 500  // Nốt dài hơn
};

void AML_Buzzer_TurnOff(void);
int AML_Buzzer_PlaySong(void);
void AML_Buzzer_PlayNote(float frequency, uint16_t duration);
//-------------------------------------------------------------------------------------------------//

void AML_Buzzer_TurnOff(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 65535);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 65535);
}

int AML_Buzzer_PlaySong(void)
{
    for (int i = 0; i < sizeof(frequencies) / sizeof(frequencies[0]); i++)
    {
        AML_Buzzer_PlayNote(frequencies[i], durations[i]);
    }
}

void AML_Buzzer_PlayNote(float frequency, uint16_t duration)
{
    // Compute the value of ARR register
    uint32_t ARR_value = (uint32_t)(ClockAfterPrescal / frequency) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim3, ARR_value);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (ARR_value / 2)); // 50% duty cycle

    // Wait for the note to play
    HAL_Delay(duration);

    // Turn off buzzer
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    AML_Buzzer_TurnOff();
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)

    // Delay between notes
    // HAL_Delay(50);
}
