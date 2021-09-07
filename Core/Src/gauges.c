#include "gauges.h"

void gaugeInit(void)
{

    for (size_t i = 0; i < 100; i++)
    {
        tempPWM[i] = 0;
        socPWM[i] = 0;
    }
}

void updateTach(volatile uint16_t amps)
{
    int freq = (amps * 100) / 315;
    TIM1->ARR = CPU / freq / TIM1->PSC;
    TIM1->CCR1 = (TIM1->ARR) / 2; //50% duty variable frequency
}

void updateSpeed(volatile uint16_t motorRPM)
{
    int freq = motorRPM / 71;
    TIM3->ARR = (CPU / freq) / TIM3->PSC;
    TIM3->CCR3 = (TIM3->ARR) / 2; //50% duty variable frequency
}

void updateTemp(volatile uint16_t temp)
{

    uint16_t highPulse = MAP(temp, 0, 90, 19, 95); //gauge range calibration

    for (size_t i = 0; i < highPulse; i++)
    {
        tempPWM[i] = (uint32_t)TEMP_Pin;
    }

    for (size_t i = highPulse; i < 100; i++)
    {
        tempPWM[i] = (uint32_t)TEMP_Pin << 16U;
    }
}

void updateSOC(volatile uint16_t soc)
{
    uint16_t highPulse = MAP(soc, 1, 100, 400, 880); //gauge range calibration
    
    for (size_t i = 0; i < highPulse; i++)
    {
        socPWM[i] = (uint32_t)SOC_Pin; 
    }

    for (size_t i = highPulse; i < 1000; i++)
    {
        socPWM[i] = (uint32_t)SOC_Pin << 16U;
    
    }
}