
#include "encoder.h"

void setWidget(uint8_t screenNum, uint8_t valueId, uint16_t currentVal, int gain)
{

    txMsgExt2.ExtId = CONTROL;
    txMsgExt2.DLC = 8;
    canTx2[0] = 0x11; // Force Widget Data
    canTx2[1] = screenNum;
    canTx2[2] = valueId;
    canTx2[3] = currentVal & 0xFF;
    canTx2[4] = (currentVal >> 8) & 0xFF;
    canTx2[5] = gain;    // Display Code placeholder
    canTx2[6] = valueId; // Value Active ID  0x00-not used, 0x01 Value 1 currently active
    canTx2[7] = 0xFF;
    // te.currentScreen = screenNum; //rely on RX values
    c2txExt(&txMsgExt2, canTx2);
}

void setBacklight(uint8_t level)
{
    txMsgExt2.ExtId = CONTROL;
    txMsgExt2.DLC = 8;
    canTx2[0] = 0x80; // Screen brightness
    canTx2[1] = 0xFF;
    canTx2[2] = level; // value 0-100
    canTx2[3] = 0xFF;
    canTx2[4] = 0xFF;
    canTx2[5] = 0xFF;
    canTx2[6] = 0xFF;
    canTx2[7] = 0xFF;
    c2txExt(&txMsgExt2, canTx2);
}

void getEvent(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{

    if (rxMsg->ExtId == EVENT)
    {
        te.currentScreen = canRx[0];
        // canRx[1] Reserved
        // canRx[2] 0x01 Events
        te.knob = canRx[3];
        te.tap = (canRx[4]) + (canRx[5] << 8);
        te.swipe = canRx[6];
    }
}

void getData(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx)
{
    if (rxMsg->ExtId == DATA)
    {
        te.currentScreen = canRx[0];
        // canRx[1] Reserved
        te.valueId = canRx[2];
        te.currentVal = canRx[3] + (canRx[4] << 8);
        te.dispCode = canRx[5];
        te.activeId = canRx[6];
    }
}

void encoderHandler(void)
{
    switch (te.currentScreen)
    {

    case HOME:
        vcu.burnFlag = OFF;
        break;

    /////// Gear Selection Screens /////////
    case FWD_REQ:

        // vcu.dio = FORWARD;
        canIOset(forward, ON);
        if (ldu.dir == FWD)
        {
            setWidget(FWD_CFM, 0, 0, 0);
        }

        break;

    case FWD_CFM:
        if (ldu.rpm >= 2000)
        {
            setWidget(HOME, 0, 0, 0);
        }
        break;

    case NTRL_REQ:
        canIOset(forward, OFF);
        canIOset(reverse, OFF);
        if (ldu.dir == NTRL)
        {
            setWidget(NTRL_CFM, 0, 0, 0);
        }
        break;

    case REV_REQ:
        // vcu.dio = REVERSE;
        canIOset(reverse, ON);
        if (ldu.dir == REV)
        {
            setWidget(REV_CFM, 0, 0, 0);
        }
        break;

    /////////// Charging Data ///////////////////////
    case BATTERY:
        // if (vcu.state != charge_keyOff || vcu.state != charge_keyOn)
        //{
        setWidget(BATTERY, BATT_VOLT, BMS[0].volt, x1);
        setWidget(BATTERY, BATT_TEMP, BMS[0].temp, x1);
        //}
        break;

    ////////// Launch Control ////////////////////////
    case LAUNCH:
        if (vcu.state != launchMode)
        {
            setWidget(HOME, 0, 0, 0);
        }
        break;

        ////////// Burnout Mode //////////////////////////////

    case BURNOUT:
        vcu.burnFlag = ON;
        break;
        /*
    case READY_BURNOUT: //HOME BUTTON DISBLED
        if (ldu.pot < 1000)
        {
            vcu.burnFlag = OFF;
            setWidget(HOME, 0, 0, 0);
        }
    */

    default:
        break;
    }

    switch (vcu.state)
    {
    case off:
        setBacklight(0);
        setWidget(HOME, 0, 0, 0);
        break;

    case on:
        setBacklight(95);
        break;

    case idle:

        switch (te.currentScreen)
        {
        case HOME:
        case WINDOWS:
        case CHARGE_STATS:
            if (charger.proximity)
            {
                setWidget(DRIVEAWAY, 0, 0, 0);
            }
            else
            {
                setWidget(NTRL_REQ, 0, 0, 0);
            }
            break;

        default:
            break;
        }

        break;

    case launchMode:
        setWidget(LAUNCH, 0, 0, 0);
        break;

    case charge_keyOff:
        setWidget(CHARGE_STATS, CS_TEMP, BMS[0].temp, x1);
        setWidget(CHARGE_STATS, CS_SOC, BMS[0].chargeState, x1);
        setWidget(CHARGE_STATS, CS_AMP, charger.current, x1);
        setBacklight(95);
        break;

    case charge_keyOn:
        setWidget(HOME, 0, 0, 0);
        setBacklight(95);
        break;
        

    default:
        setBacklight(95);
        break;
    }
}
