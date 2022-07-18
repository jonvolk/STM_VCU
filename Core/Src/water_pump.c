#include "water_pump.h"

void wpHandler(void)
{
    switch (vcu.state)
    {
    case run:
        if (ldu.dir == FWD || ldu.dir == REV)
        {
            int throttleDuty = MAP(ldu.pot, 870, 4095, 100, 100);
            int tempDuty = MAP(ldu.hsTemp, 0, 85, 100, 100);

            if (tempDuty > throttleDuty)
            {
                wpDuty(tempDuty);
            }
            else
            {
                wpDuty(throttleDuty);
            }
        }


        
        else
        {
            wpDuty(0);
        }

        break;

    case charge_keyOff:
    case charge_keyOn:
        wpDuty(100);   
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