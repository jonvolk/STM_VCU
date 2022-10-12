#include "water_pump.h"

void wpHandler(void)
{
    uint8_t fanDuty = 0;
    switch (vcu.state)
    {
    case run:
    case launchMode:
    case burnout:
        if (ldu.dir == FWD || ldu.dir == REV)
        {

            if (ldu.hsTemp > 40 || BMS[0].temp > 95)
            {
                fanDuty = 100;
            }
            else
            {
                fanDuty = 0;
            }

            wpDuty(100);

            txMsg.StdId = 0x140;
            txMsg.DLC = 8;
            canTx[0] = fanDuty; // run fans at 30%
            canTx[3] = 95;      // run water pump at 80%
            c1tx(&txMsg, canTx);
        }

        else
        {
            wpDuty(0);
        }

        break;

    case charge_keyOff:
    case charge_keyOn:
        wpDuty(100);

        if (BMS[0].temp > 95)
        {
            fanDuty = 40;
        }
        txMsg.StdId = 0x140;
        txMsg.DLC = 8;
        canTx[0] = fanDuty; // run fans at 30%
        canTx[3] = 95;      // run water pump at 80%
        c1tx(&txMsg, canTx);
        break;


    case off:
        wpDuty(0);
    default:
        break;
    }
}

void wpInit(void)
{
    for (size_t i = 0; i < 100; i++)
    {
        waterPWM[i] = 0;
    }
}

void wpDuty(volatile int duty)
{
    for (size_t i = 0; i < duty; i++)
    {
        waterPWM[i] = (uint32_t)WP_Pin;
    }

    for (size_t i = duty; i < 100; i++)
    {
        waterPWM[i] = (uint32_t)WP_Pin << 16U;
    }
}