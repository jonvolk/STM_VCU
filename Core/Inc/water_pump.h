#if !defined(__WATER_PUMP_H_)
#define __WATER_PUMP_H_

#include "main.h"
#include "vcu.h"
#include "my_math.h"
#include "can_setup.h"

uint32_t waterPWM[100];

void wpInit(void);
void wpHandler(void);
void wpDuty(volatile int duty);

#endif // __WATER_PUMP_H_

