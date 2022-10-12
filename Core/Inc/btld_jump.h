#if !defined(BOOTLOAD_JUMP_H__)
#define BOOTLOAD_JUMP_H__

#include "can_setup.h"

void jumpToBootloader(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);

#endif // BOOTLOAD_JUMP_H__
