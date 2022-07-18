#if !defined(__GAUGES_H_)
#define __GAUGES_H_

//#include "stm32f105xc.h"
#include "main.h"
#include "my_math.h"

#define CPU 72000000

uint32_t tempPWM[100];
uint32_t socPWM[1000];

void gaugeInit(void);
void updateTach(volatile uint16_t amps);
void updateSpeed(volatile uint16_t motorRPM);
void updateTemp(volatile uint16_t temp);
void updateSOC(volatile uint16_t soc);

#endif // __GAUGES_H_
