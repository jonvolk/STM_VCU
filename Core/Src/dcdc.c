#include "dcdc.h"


void dcdcInit(void)
{
    time_now = HAL_GetTick();
    period = 1200000; // 20 minute recharge period
    tenderActive = false;

}

void dcdc_OFF(void)
{
    txMsg.StdId = 0x1D4;
    txMsg.DLC = 2;
    canTx[0] = 0x00;
    canTx[1] = 0x00;
    c1tx(&txMsg, canTx);
}

void dcdc_ON(void)
{
    txMsg.StdId = 0x1D4;
    txMsg.DLC = 2;
    canTx[0] = 0xA0;
    canTx[1] = 0xBA;
    c1tx(&txMsg, canTx);
}

void dcdc_DATA(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{
    if (rxMsg->StdId == 0x1D6)
    {
        DCDC.volts = (canRx[2] / 12); // 12 was 12.7
        DCDC.temp1 = (((canRx[3] - 40) * 1) + 32); //1 was 1.8 
        DCDC.temp2 = (((canRx[4] - 40) * 1) + 32); //1 was 1.8
        DCDC.current = canRx[5];
    }
}

void dcdcHandler(int vehicleState)
{
    switch (vehicleState)
    {
    case off:
        if (ADC_data[lvread] < 3600) //~12.1vdc
        {
            tenderActive = true;
            time_now = HAL_GetTick();
        }
        if(tenderActive)
        {
            dcdc_ON();
            if ((HAL_GetTick() - time_now) > period)
            {
                //time_now = HAL_GetTick();
                tenderActive = false;
            }
        }
        if (!tenderActive)
        {
            dcdc_OFF();
        }
        break;

    case on:
    case run:
    case charge_keyOn:
    case charge_keyOff:
    case idle:
    case launchMode:
        dcdc_ON();
        break; 

    default:
        break;
    }
}