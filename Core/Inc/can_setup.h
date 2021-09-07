
#ifndef __CAN_SETUP_H
#define __CAN_SETUP_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdio.h"
#include "vcu.h"
#include "encoder.h"
#include "dcdc.h"

//CAN_HandleTypeDef hcan;
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_FilterTypeDef sf;  //= {0};  //CAN Bus Filter
CAN_FilterTypeDef sf2; // = {0}; //CAN2 Bus Filter
CAN_FilterTypeDef sf3;
CAN_FilterTypeDef sf4;
CAN_FilterTypeDef sf5;

CAN_RxHeaderTypeDef rxMsg; //CAN Bus Receive Header
CAN_TxHeaderTypeDef txMsg; //CAN Bus Transmit Header
CAN_TxHeaderTypeDef txMsgExt;

CAN_RxHeaderTypeDef rxMsg2; //CAN2 Bus Receive Header
CAN_TxHeaderTypeDef txMsg2; //CAN2 Bus Transmit Header
CAN_TxHeaderTypeDef txMsgExt2;

uint8_t canRx[8]; //= {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t canRx2[8];

uint32_t canMailbox; //CAN Bus Mail box variable
uint32_t canMailbox2;

uint8_t canTx[8];
uint8_t canTx2[8];

//int busCount; // SINGLE or DUAL
int txCycle; //CAN TX counter
int txCycle2;
int canDebug;

void canSettings(void);
void c1tx(CAN_TxHeaderTypeDef *txMsg, uint8_t *canTx);
void c2tx(CAN_TxHeaderTypeDef *txMsg2, uint8_t *canTx2);
void c1txExt(CAN_TxHeaderTypeDef *txMsgExt, uint8_t *canTx);
void c2txExt(CAN_TxHeaderTypeDef *txMsg2Ext, uint8_t *canTx2);
void canTxDelay(void);

#endif
