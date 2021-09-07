#include "vcu.h"

void decodeCAN(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{

    switch (rxMsg->StdId)
    {
    case 0x135:
        if ((((canRx[3] << 8) + canRx[2])) <= 2000)
        {
            ldu.amps = (((canRx[3] << 8) + canRx[2]) * 1.83);
        }
        else if ((((canRx[3] << 8) + canRx[2])) >= 3000)
        {
            ldu.amps = (((((canRx[3] << 8) + canRx[2]) - 65535) * 1.83) * -1);
        }
        /**************************************************************************/
        ldu.rpm = (((canRx[1] << 8) + canRx[0]));
        /**************************************************************************/
        if ((canRx[4]) > 0)
        {
            ldu.mTemp = (canRx[5]); //motor temp C
        }
        /**************************************************************************/
        if ((canRx[5]) > 0)
        {
            ldu.hsTemp = (canRx[4]); //heatsink temp C
        }
        /**************************************************************************/
        if ((((canRx[7] << 8)) + canRx[6]) <= 2000)
        {
            ldu.potNom = (((canRx[7] << 8)) + canRx[6]);
        }
        else if ((((canRx[7] << 8)) + canRx[6]) >= 2000)
        {
            ldu.potNom = ((((canRx[7] << 8)) + canRx[6]) - 65535);
        }
        break;

    case 0x04F:
        ldu.dir = canRx[0];
        ldu.brake = canRx[1];
        break;

    case 0x136:
        ldu.mode = (canRx[0]);
        break;

    case 0x109:
        charger.chargerStatus = (canRx[5]);
        charger.current = (canRx[3] * 100) / 55;
        break;

    case 0x113:
        ldu.pot = ((canRx[1] << 8) + canRx[0]);
        ldu.pot2 = ((canRx[3] << 8) + canRx[2]);
        break;

    case 0x12D:
        //restart = ((canRx[1] << 8) + canRx[0]);
        break;
    case 0x38E:
        iboost.pedal = ((canRx[4] << 8) + (canRx[3])) - 4415; //0-750
        break;

    case 0x138:
        BMS[0].chargeRequest = canRx[0];
        BMS[0].volt = ((canRx[2] << 8) + canRx[1]) / 100;
        BMS[0].temp = canRx[3];
        BMS[0].highCell = (canRx[4]) / 51;
        BMS[0].lowCell = (canRx[5]) / 51;
        BMS[0].chargeState = canRx[6];
        BMS[0].soc = canRx[7];
        break;

    case 0x139:
        BMS[1].chargeRequest = canRx[0];
        BMS[1].volt = ((canRx[2] << 8) + canRx[1]) / 100;
        BMS[1].temp = canRx[3];
        BMS[1].highCell = (canRx[4]) / 51;
        BMS[1].lowCell = (canRx[5]) / 51;
        BMS[1].chargeState = canRx[6];
        BMS[1].soc = canRx[7];
        break;

    default:
        break;
    }
}

void canIOset(int bit, int val)
{
    if ((val = ON))
    {
        vcu.dio |= (1U << (bit));
    }
    else if ((val = OFF))
    {
        vcu.dio &= ~(1U << (bit));
    }

    txMsg.StdId = 0x113;
    txMsg.DLC = 1;
    canTx[0] = vcu.dio;
    c1tx(&txMsg, canTx);
}

void canIOsend(void)
{
    txMsg.StdId = 0x113;
    txMsg.DLC = 1;
    canTx[0] = vcu.dio;
    c1tx(&txMsg, canTx);
}
/////////////////////////////////////////////////////////////////////////
void vcuState(void)
{
    switch (vcu.state)
    {
    case off:

        if (vcu.key == ON && charger.chargerStatus == CHRG_ENABLED)
        {
            vcu.state = charge_keyOn;
        }
        if (vcu.key == OFF && charger.chargerStatus == CHRG_ENABLED)
        {
            vcu.state = charge_keyOff;
        }
        if (vcu.key == ON && charger.chargerStatus == OFF)
        {
            vcu.state = on;
        }
        break;

    case on:
        canSet(IDLE_MODE, 1, 32);
        if (ldu.mode == 1)
        {
            vcu.state = idle;
        }

        if (vcu.key == OFF)
        {
            vcu.state = off;
        }
        if (charger.chargerStatus == 5)
        {
            vcu.state = charge_keyOn;
        }
        break;

    case idle:
        canSet(IDLE_MODE, 1, 32);
        if (ldu.dir != 2)
        {
            vcu.state = run;
        }

        if (vcu.key == OFF)
        {
            vcu.state = off;
        }
        break;

    case run:
        canSet(IDLE_MODE, 0, 32);
        canSet(FWEAK, 280, 32);
        canSet(FSLIP_MIN, 76, 1);  // 2.3*32
        canSet(FSLIP_MAX, 101, 1); // 3.15*32
        if (vcu.key == OFF)
        {
            vcu.state = off;
        }

        if (vcu.launchFlag == ON)
        {
            vcu.state = launchMode;
        }
        break;

    case launchMode:
        canSet(FWEAK, 220, 32);
        canSet(FSLIP_MIN, 61, 1); // 1.9*32
        canSet(FSLIP_MAX, 77, 1); // 2.4*32
        if (vcu.launchFlag == 0)
        {
            vcu.state = run;
        }
        break;

    default:
        break;
    }
}

void vcuInit(void)
{
    vcu.state = off;
    vcu.dio = 0;
    vcu.key = 0;
    vcu.launchFlag = 0;
    ADC_data[0] = 4095; // LVREAD PIN
    ADC_data[1] = 0;
    ADC_data[2] = 0;
}

void ioHandler(void)
{
    ////////////////  Key Input ///////////////////////////////////////
    if (HAL_GPIO_ReadPin(SW12_GPIO_Port, SW12_Pin))
    {
        vcu.key = ON;
    }
    else
    {
        vcu.key = OFF;
        vcu.dio = 0x00;
        ldu.mode = 0;
        if (charger.chargerStatus != CHRG_ENABLED)
        {
            vcu.state = off;
        }
    }

    //////////////////  Heater Switching ///////////////////////////////
    int heatRequest = HAL_GPIO_ReadPin(HEAT_REQ_GPIO_Port, HEAT_REQ_Pin);

    switch (vcu.state)
    {
    case on:
        if (heatRequest)
        {
            HAL_GPIO_WritePin(HEAT_OUT_GPIO_Port, HEAT_OUT_Pin, ON);
        }
        else
        {
            HAL_GPIO_WritePin(HEAT_OUT_GPIO_Port, HEAT_OUT_Pin, OFF);
        }
        break;

    case off:
        HAL_GPIO_WritePin(HEAT_OUT_GPIO_Port, HEAT_OUT_Pin, OFF);
        break;

    default:
        break;
    }

    //////////////////  Power Steering  ///////////////////////////////
    switch (vcu.state)
    {
    case off:
        HAL_GPIO_WritePin(PS_INIT_GPIO_Port, PS_INIT_Pin, OFF);
        break;
    case run:
    case idle:
        HAL_GPIO_WritePin(PS_INIT_GPIO_Port, PS_INIT_Pin, ON);
        break;

    default:
        break;
    }
}

void regenHandler(void)
{

    int baseRegen = 0; //base throttle off regen value
    int maxRegen = 94; //maximum full brake pressure regen value
    int brkNomPedal;
    int regenRamp;

    if (iboost.pedal > 700)
    {
        brkNomPedal = -(maxRegen);
    }
    else
    {
        brkNomPedal = MAP(iboost.pedal, 1, 700, baseRegen, -(maxRegen)); 
        ; //maps brake pedal regen between base and max
    }
    canSet(BRAKE_NOM_PEDAL, brkNomPedal, 32);

    //regenramp
    if (ldu.rpm <= 10000)
    {
        regenRamp = MAP(ldu.rpm, 0, 10000, 2, 12); // mapped values alredy 32x for gain
    }
    else
    {
        regenRamp = 12; // value already 32x for gain
        canSet(BRAKE_PEDAL_RAMP, regenRamp, 1);
    }
}

void canSet(uint8_t index, uint32_t value, uint8_t gain) // LDU param Index, unscaled value, value already *32
{
    int val = value * gain;

    txMsg.StdId = 0x601; //set parameter ID
    txMsg.DLC = 8;
    canTx[0] = 0x40;
    canTx[1] = 0x00;
    canTx[2] = 0x20;
    canTx[3] = index;
    canTx[4] = val & 0xFF;
    canTx[5] = (val >> 8) & 0xFF;
    canTx[6] = (val >> 16) & 0xFF;
    canTx[7] = (val >> 24) & 0xFF;
    c1tx(&txMsg, canTx);

}

void throttleHandler(void)
{
    ///////// idle throttle ///////////////////
    int idleThrotMax;
    int idleThrot;

    if (ldu.dir == FWD)
    {
        idleThrotMax = 20;
    }
    else
    {
        idleThrotMax = 18;
    }

    idleThrot = MAP(iboost.pedal, 1, 400, idleThrotMax, 0);
    canSet(IDLE_THROT_LIM, idleThrot, 32);

    ///////// Launch Control Enable ///////////////
    if (ldu.pot >= 4050 && ldu.brake == ON)
    {
        vcu.launchFlag = ON;
    }
    if (vcu.launchFlag == ON && ldu.pot < 3000)
    {
        vcu.launchFlag = OFF;
    }
}