#include "dynamics.h"

int zero2sixty(void)
{
    return GPS.lon;
}

int getPitch(void)
{
    return GPS.lat;
}

void getDynamics(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{
    switch (rxMsg->ExtId)
    {
    case 0x0000A0000:
        GPS.lat = (canRx[0] << 24) + (canRx[1] << 16) + (canRx[2] << 8) + canRx[3];
        GPS.lon = (canRx[4] << 24) + (canRx[5] << 16) + (canRx[6] << 8) + canRx[7];
        break;

    case 0x0000A0001:
        GPS.speed = (canRx[0] << 8) + canRx[1];
        GPS.alt = (canRx[2] << 8) + canRx[3];
        GPS.course = (canRx[4] << 8) + canRx[5];
        GPS.speed = canRx[6];
        GPS.valid_10hz = canRx[7];
        break;

    case 0x0000A0002:
        UTC.valid_5hz = canRx[0];
        UTC.year = canRx[1];
        UTC.month = canRx[2];
        UTC.day = canRx[3];
        UTC.hours = canRx[5];
        UTC.minutes = canRx[6];
        UTC.seconds = canRx[7];
        break;

    case 0x0000A0003: // val*0x000244141 +8g to -8g
        ACCEL.x = (canRx[0] << 8) + canRx[1];
        ACCEL.y = (canRx[2] << 8) + canRx[3];
        ACCEL.z = (canRx[4] << 8) + canRx[5];
        break;

    case 0x0000A0004: // val*0x015258789 +500deg/s to -500deg/s
        YAW.x = (canRx[0] << 8) + canRx[1];
        YAW.y = (canRx[2] << 8) + canRx[3];
        YAW.z = (canRx[4] << 8) + canRx[5];
        break;

    default:
        break;
    }
}