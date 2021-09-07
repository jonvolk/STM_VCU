
#if !defined(__DCDC_H__)
#define __DCDC_H__

#include "can_setup.h"
#include "stdbool.h"
//#include "vcu.h"
#define TEND_TIME 900000  


typedef struct
{
    uint8_t volts;
    uint8_t temp1;
    uint8_t temp2;
    uint8_t current;

} dcdc_t;
dcdc_t DCDC;

uint32_t period;// = 1000; // Tender on time in milliseconds
uint32_t time_now;// = 0; //Timekeeper
bool tenderActive;// = 0;
//int ADC_placeholder;

void dcdc_OFF(void);
void dcdc_ON(void);
void dcdcHandler(int vehicleState);
void dcdc_DATA(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void dcdcInit(void);

#endif // __DCDC_H__
