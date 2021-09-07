#include <can_setup.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{

    CAN_RxHeaderTypeDef rxMsg;
    uint8_t canRx[8];

    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxMsg, canRx) != HAL_OK)
    {
        Error_Handler();
    }
    //do stuff
    decodeCAN(&rxMsg, canRx);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan2)
{

    CAN_RxHeaderTypeDef rxMsg2;
    uint8_t canRx2[8];

    if (HAL_CAN_GetRxMessage(hcan2, CAN_RX_FIFO1, &rxMsg2, canRx2) != HAL_OK)
    {
        Error_Handler();
    }
    //do stuff
    getData(&rxMsg2, canRx2);
    getEvent(&rxMsg2, canRx2);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void canSettings(void)
{

    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.TransmitGlobalTime = DISABLE;

    txMsgExt.IDE = CAN_ID_EXT;
    txMsgExt.RTR = CAN_RTR_DATA;
    txMsgExt.TransmitGlobalTime = DISABLE;

    sf.FilterBank = 0; // CAN1 Filter bank starts at 0
    sf.FilterMode = CAN_FILTERMODE_IDLIST;
    sf.FilterScale = CAN_FILTERSCALE_16BIT;
    sf.FilterIdLow = 0x1D6 << 5;      //DCDC DATA
    sf.FilterIdHigh = 0x04F << 5;      //LDU DIR BRAKE
    sf.FilterMaskIdLow = 0x113 << 5;  //LDU POT1 POT2
    sf.FilterMaskIdHigh = 0x131 << 5; //LDU DIO
    sf.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf.SlaveStartFilterBank = 14;
    sf.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
    {
        Error_Handler();
    }

    sf4.FilterBank = 1; // CAN1 Filter bank starts at 0
    sf4.FilterMode = CAN_FILTERMODE_IDLIST;
    sf4.FilterScale = CAN_FILTERSCALE_16BIT;
    sf4.FilterIdLow = 0x135 << 5;      //LDU AMPS,RPM,HSTEMP,POTNOM
    sf4.FilterIdHigh = 0x136 << 5;     //LDU PACK VOLT, RUN
    sf4.FilterMaskIdLow = 0x138 << 5;  //BMS1
    sf4.FilterMaskIdHigh = 0x139 << 5; //BMS2
    sf4.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf4.SlaveStartFilterBank = 14;
    sf4.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &sf4) != HAL_OK)
    {
        Error_Handler();
    }

    sf5.FilterBank = 2; // CAN1 Filter bank starts at 0
    sf5.FilterMode = CAN_FILTERMODE_IDLIST;
    sf5.FilterScale = CAN_FILTERSCALE_16BIT;
    sf5.FilterIdLow = 0x109 << 5;      //CHARGER STATUS
    sf5.FilterIdHigh = 0x38E << 5;     //IBOOST
    sf5.FilterMaskIdLow = 0x581 << 5;  //CANOPEN
    sf5.FilterMaskIdHigh = 0x601 << 5; //CANOPEN
    sf5.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf5.SlaveStartFilterBank = 14;
    sf5.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &sf5) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);

    //hcan2
    txMsg2.IDE = CAN_ID_STD;
    txMsg2.RTR = CAN_RTR_DATA;
    txMsg2.TransmitGlobalTime = DISABLE;

    txMsgExt2.IDE = CAN_ID_EXT;
    txMsgExt2.RTR = CAN_RTR_DATA;
    txMsgExt2.TransmitGlobalTime = DISABLE;

    sf2.FilterBank = 14; // CAN2 Filter bank starts at 14
    sf2.FilterMode = CAN_FILTERMODE_IDLIST;
    sf2.FilterScale = CAN_FILTERSCALE_32BIT;
    sf2.FilterIdLow = ((0x18FF11F2 << 3) & 0xFFF8) | 4; //ENCODER EVENT
    sf2.FilterIdHigh = (0x18FF11F2 >> 13) & 0xFFFF;
    sf2.FilterMaskIdLow = ((0x18FF0FF2 << 3) & 0xFFF8) | 4; //ENCODER DATA
    sf2.FilterMaskIdHigh = (0x18FF0FF2 >> 13) & 0xFFFF;
    sf2.FilterFIFOAssignment = CAN_RX_FIFO1;
    sf2.SlaveStartFilterBank = 14;
    sf2.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan2, &sf2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printCAN1(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{

    uint16_t ID = (rxMsg->StdId);
    uint64_t xID = (rxMsg->ExtId);
    int IDE = (rxMsg->IDE);
    int dataLength = (rxMsg->DLC);

    printf("\r\n");

    if (IDE == 0)
    {
        printf("CAN_1, %ld, 0x%.3X,", HAL_GetTick(), ID);
    }
    else if (IDE == 4)
    {
        printf("CAN_1, %ld, 0x%.8llX,", HAL_GetTick(), xID);
    }
    printf("%d,", dataLength);

    for (int i = 0; i < dataLength; i++)
        printf(" ,0x%.2X", canRx[i]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printCAN2(CAN_RxHeaderTypeDef *rxMsg2, uint8_t *canRx2)
{

    uint16_t ID = (rxMsg2->StdId);
    uint64_t xID = (rxMsg2->ExtId);
    int IDE = (rxMsg2->IDE);
    int dataLength = (rxMsg2->DLC);

    printf("\r\n");
    if (IDE == 0)
    {
        printf("CAN_2, %ld, 0x%.3X,", HAL_GetTick(), ID);
    }
    else if (IDE == 4)
    {
        printf("CAN_2, %ld, 0x%.8llX,", HAL_GetTick(), xID);
    }
    printf("%d,", dataLength);

    for (int i = 0; i < dataLength; i++)
        printf(" ,0x%.2X", canRx[i]);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void c1tx(CAN_TxHeaderTypeDef *txMsg, uint8_t *canTx)
{

    HAL_CAN_AddTxMessage(&hcan1, txMsg, canTx, &canMailbox);
    txCycle++;
    if (txCycle >= 3)
    {
        HAL_Delay(1);
        txCycle = 0;
    }
}

void c2tx(CAN_TxHeaderTypeDef *txMsg2, uint8_t *canTx2)
{

    HAL_CAN_AddTxMessage(&hcan2, txMsg2, canTx2, &canMailbox2);
    txCycle2++;
    if (txCycle2 >= 3)
    {
        HAL_Delay(1);
        txCycle2 = 0;
    }
}

void c1txExt(CAN_TxHeaderTypeDef *txMsgExt, uint8_t *canTx)
{

    HAL_CAN_AddTxMessage(&hcan1, txMsgExt, canTx, &canMailbox);
    txCycle++;
    if (txCycle >= 3)
    {
        HAL_Delay(1);
        txCycle = 0;
    }
}

void c2txExt(CAN_TxHeaderTypeDef *txMsg2Ext, uint8_t *canTx2)
{

    HAL_CAN_AddTxMessage(&hcan2, txMsg2Ext, canTx2, &canMailbox2);
    txCycle2++;
    if (txCycle2 >= 3)
    {
        HAL_Delay(1);
        txCycle2 = 0;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/***************** Filter Config ID mask: Allow All *******************
    sf.FilterBank = 0; // CAN1 Filter bank starts at 0 
    sf.FilterMode = CAN_FILTERMODE_IDMASK;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterIdLow = 0xffff;  
    sf.FilterIdHigh = 0x1fff;
    sf.FilterMaskIdLow = 0x0000;
    sf.FilterMaskIdHigh = 0x0000;
    sf.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf.SlaveStartFilterBank = 14;
    sf.FilterActivation = ENABLE;
*/

/***************** Filter Config ID list: Allow 4 discreet ID *******************
    sf1.FilterBank = 1;
    sf1.FilterMode = CAN_FILTERMODE_IDLIST; 
    sf1.FilterScale = CAN_FILTERSCALE_16BIT;
    sf1.FilterIdLow = 0xFD3<<5; 
    sf1.FilterIdHigh = 0X120<<5;
    sf1.FilterMaskIdLow = 0x184<<5;//X120<<5;
    sf1.FilterMaskIdHigh = 0x084<<5;//X120<<5;
    sf1.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf1.SlaveStartFilterBank = 14;
    sf1.FilterActivation = ENABLE;
*/

/***************** Filter Config EXTID list: Allow 2 discreet ID *******************
    sf.FilterBank = 0;
    sf.FilterMode = CAN_FILTERMODE_IDLIST;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterIdLow = ((0x18FF11F2 << 3) & 0xFFF8) | 4;   
    sf.FilterIdHigh = (0x18FF11F2 >> 13) & 0xFFFF;         
    sf.FilterMaskIdLow = ((0x18FF0FF2 << 3) & 0xFFF8) | 4; 
    sf.FilterMaskIdHigh = (0x18FF0FF2 >> 13) & 0xFFFF;     
    sf.FilterFIFOAssignment = CAN_RX_FIFO0;
    sf.SlaveStartFilterBank = 14;
    sf.FilterActivation = ENABLE;
*/