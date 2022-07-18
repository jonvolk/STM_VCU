#if !defined(__LIDAR_H__)
#define __LIDAR_H__

#include "main.h"

typedef struct 
{
    uint16_t distance;
    uint16_t signal;
    uint32_t time;
}lidar_t;
lidar_t LIDAR;

void getLidar(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);


#endif // __LIDAR_H__
