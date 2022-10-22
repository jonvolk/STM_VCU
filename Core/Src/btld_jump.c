#include "btld_jump.h"

#define BOOTLOADER_ID 0x0000FF00
#define BTLD_APP_ID 0X01
#define JUMP_COMMAND 0XF0
#define BOOT_ADDRESS (uint32_t)0x08000000 /* Start address of application space in flash */

typedef void (*pFunction)(void);
static void btld_JumpToBoot(void);

void jumpToBootloader(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{
    //if ((rxMsg->ExtId == (BOOTLOADER_ID | BTLD_APP_ID)) && canRx[0] == JUMP_COMMAND)
    if (rxMsg->ExtId == 0x0000FF01)
    {
        //btld_JumpToBoot();
        HAL_NVIC_SystemReset();
    }
}

void btld_JumpToBoot(void)
{

    uint32_t JumpAddress = *(__IO uint32_t *)(BOOT_ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;

    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

#if (SET_VECTOR_TABLE)
    SCB->VTOR = APP_ADDRESS;
#endif

    __set_MSP(*(__IO uint32_t *)BOOT_ADDRESS);
    Jump();
}