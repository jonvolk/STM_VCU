#ifndef __PRINT_UART_H
#define __PRINT_UART_H


#include <stdio.h>
#include "stm32f1xx_hal.h"

UART_HandleTypeDef huart1;

//////////////////////////// Begin UART Magic ////////////////////////
int __io_putchar(int ch)
{
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit(&huart1, &*c, 1, 10);
 return ch;
}

int _write(int file,char *ptr, int len)
{
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
return len;
}
//////////////////////////////////// END UART Magic //////////////////////

#endif