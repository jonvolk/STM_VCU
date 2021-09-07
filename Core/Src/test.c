#include "test.h"

void testVal(void)
{
#ifdef ACTIVE

    if (ldu.amps < 1200)
    {
        ldu.amps += 5;
    }

    if (ldu.hsTemp < 90)
    {
        ldu.hsTemp++;
    }

    if (ldu.rpm < 18000)
    {
        ldu.rpm += 20;
    }

    if (ldu.SOC < 100)
    {
        ldu.SOC ++;
    }

#endif // ACTIVE 

#ifdef PASSIVE
    ldu.amps = 600;
    ldu.hsTemp = 40;
    ldu.rpm = 4500;
    BMS[0].soc = 55;
#endif //PASSIVE
}
