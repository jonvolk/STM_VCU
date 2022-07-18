#include "lidar.h"

void getLidar(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{
    if (rxMsg->ExtId == 0x00000003)
    {
        LIDAR.distance = (canRx[1] << 8) + canRx[0];
        LIDAR.signal = (canRx[3] << 8) + canRx[2];
        LIDAR.time = (canRx[7] << 24) + (canRx[6] << 16) + (canRx[5] << 8) + canRx[4];
    }
}