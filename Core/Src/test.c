#include "test.h"

void testVal(void)
{
#ifdef ACTIVE

    if (ldu.amps < 1200)
    {
        ldu.amps += 5;
    }
    if (ldu.amps >= 1200)
    {
        ldu.amps = 0;
    }

    if (ldu.rpm < 18000)
    {
        ldu.rpm += 70;
    }
    if (ldu.rpm >= 18000)
    {
        ldu.rpm = 0;
    }

    ldu.hsTemp = 60;
    BMS[0].chargeState = 55;

#endif // ACTIVE

#ifdef PASSIVE
    ldu.amps = 600;
    ldu.hsTemp = 40;
    ldu.rpm = 4500;
    BMS[0].soc = 55;
#endif //PASSIVE
}
