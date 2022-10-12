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
            ldu.mTemp = (canRx[5]); // motor temp C
        }
        /**************************************************************************/
        if ((canRx[5]) > 0)
        {
            ldu.hsTemp = (canRx[4]); // heatsink temp C
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
        ldu.pot = ((canRx[5] << 8) + canRx[4]);
        break;

    case 0x109:
        charger.chargerStatus = (canRx[5]);
        charger.current = (canRx[3] * 100) / 55;
        break;

    case 0x110:
        charger.proximity = (canRx[0]);
        break;
        /*
            case 0x136:
                ldu.pot = ((canRx[1] << 8) + canRx[0]);
                ldu.pot2 = ((canRx[3] << 8) + canRx[2]);
                break;
        */
    case 0x12D:
        // restart = ((canRx[1] << 8) + canRx[0]);
        break;
    case 0x38E:
        iboost.pedal = ((canRx[4] << 8) + (canRx[3])) - 4415; // 0-750
        break;

    case 0x138:
        // BMS[0].chargeRequest = canRx[0];
        BMS[0].volt = ((canRx[1] << 8) + canRx[0]) / 100;
        BMS[0].temp = ((canRx[3] << 8) + canRx[2]) / 100; // canRx[3];
        BMS[0].highCell = (canRx[4]) / 100;
        BMS[0].lowCell = (canRx[5]) / 100;
        BMS[0].chargeState = canRx[6];
        BMS[0].soc = canRx[7];
        break;

    case 0x139:
        // BMS[1].chargeRequest = canRx[0];
        BMS[1].volt = ((canRx[1] << 8) + canRx[0]) / 100;
        BMS[1].temp = ((canRx[3] << 8) + canRx[2]) / 100; // canRx[3];
        BMS[1].highCell = (canRx[4]) / 100;
        BMS[1].lowCell = (canRx[5]) / 100;
        BMS[1].chargeState = canRx[6];
        BMS[1].soc = canRx[7];
        break;

    default:
        break;
    }
}

//////////////// LDU DIO FUNCTIONS /////////////////////////////
void canIOset(int bit, int val)
{
    if (val)
    {
        vcu.dio |= (1U << (bit));
    }
    else
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
void vehicleComms(void)
{
    txMsg.StdId = 0x313;
    txMsg.DLC = 8;
    canTx[0] = vcu.state;
    canTx[1] = getResetCause();
    canTx[2] = taskTime.Task10ms & 0xFF;
    canTx[3] = (taskTime.Task10ms >> 8) & 0xFF;
    canTx[4] = taskTime.TaskLoop & 0xFF;
    canTx[5] = (taskTime.TaskLoop >> 8) & 0xFF;
    canTx[6] = taskTime.Task250ms & 0xFF;
    canTx[7] = (taskTime.Task250ms >> 8) & 0xFF;
    c1tx(&txMsg, canTx);

    txMsg.StdId = 0x314;
    txMsg.DLC = 8;
    canTx[0] = taskTime.TaskLoop_max & 0xFF;
    canTx[1] = (taskTime.TaskLoop_max >> 8) & 0xFF;
    canTx[2] = taskTime.Task10ms_max & 0xFF;
    canTx[3] = (taskTime.Task10ms_max >> 8) & 0xFF;
    canTx[4] = taskTime.Task100ms_max & 0xFF;
    canTx[5] = (taskTime.Task100ms_max >> 8) & 0xFF;
    canTx[6] = taskTime.Task250ms_max & 0xFF;
    canTx[7] = (taskTime.Task250ms_max >> 8) & 0xFF;
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
        canSet(FWEAKSTRT, 400, 32);
        canSet(FSLIP_MIN, 76, 1);  // 2.3*32
        canSet(FSLIP_MAX, 101, 1); // 3.15*32
        canSet(THROTRAMP, 15, 32);
        if (vcu.key == OFF)
        {
            vcu.state = off;
        }

        if (vcu.launchFlag)
        {
            vcu.state = launchMode;
        }

        if (vcu.burnFlag)
        {
            vcu.state = burnout;
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

    case burnout:
        canSet(FWEAK, 220, 32);
        canSet(FWEAKSTRT, 238, 32);
        canSet(FSLIP_MIN, 61, 1); // 1.9*32
        canSet(FSLIP_MAX, 77, 1); // 2.4*32
        int throttleramp;
        if (ldu.rpm < 2000)
        {
            throttleramp = 2;
        }
        else
        {
            throttleramp = MAP(ldu.rpm, 2000, 16000, 2, 30);
        }
        canSet(THROTRAMP, throttleramp, 32); // canSet(THROTRAMP, 5, 32);
        if (vcu.burnFlag == OFF)
        {
            vcu.state = run;
        }
        break;

    case charge_keyOn:
        if (vcu.key == OFF)
        {
            vcu.state = off;
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
    vcu.burnFlag = 0;
    iboost.pedal = 700;
    ADC_data[0] = 4095; // LVREAD PIN
    ADC_data[1] = 0;
    ADC_data[2] = 0;
    taskTime.TaskLoop = 0;
    taskTime.TaskLoop_max = 0;
    taskTime.Task10ms = 0;
    taskTime.Task10ms_max = 0;
    taskTime.Task100ms = 0;
    taskTime.Task100ms_max = 0;
    taskTime.Task250ms = 0;
    taskTime.Task250ms_max = 0;
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
    case off:
    case charge_keyOff:
        HAL_GPIO_WritePin(HEAT_OUT_GPIO_Port, HEAT_OUT_Pin, OFF);
        break;

    default:
        if (heatRequest)
        {
            HAL_GPIO_WritePin(HEAT_OUT_GPIO_Port, HEAT_OUT_Pin, ON);
        }
        else
        {
            HAL_GPIO_WritePin(HEAT_OUT_GPIO_Port, HEAT_OUT_Pin, OFF);
        }
        break;
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

    int baseRegen = 5; // base throttle off regen value
    int maxRegen = 94; // maximum full brake pressure regen value
    int brkNomPedal;
    int regenRamp;

    if (iboost.pedal > 650)
    {
        brkNomPedal = -(maxRegen);
    }
    else
    {
        brkNomPedal = MAP(iboost.pedal, 1, 650, baseRegen, -(maxRegen));
        ; // maps brake pedal regen between base and max
    }
    canSet(BRAKE_NOM_PEDAL, brkNomPedal, 32);

    // regenramp
    if (ldu.rpm <= 10000)
    {
        regenRamp = MAP(ldu.rpm, 0, 10000, 6, 24); // mapped values alredy 32x for gain  was 2, 12
    }
    else
    {
        regenRamp = 24; // value already 32x for gain
        canSet(BRAKE_PEDAL_RAMP, regenRamp, 1);
    }
}

void canSet(uint8_t index, uint32_t value, uint8_t gain) // LDU param Index, unscaled value, value already *32
{
    int val = value * gain;

    txMsg.StdId = 0x601; // set parameter ID
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
        idleThrotMax = 22;
    }
    else
    {
        idleThrotMax = 18;
    }

    if (iboost.pedal > 400)
    {
        canSet(IDLE_THROT_LIM, 0, 32);
    }
    else
    {
        idleThrot = MAP(iboost.pedal, 1, 400, idleThrotMax, 0);
        canSet(IDLE_THROT_LIM, idleThrot, 32);
    }

    ///////// Launch Control Enable ///////////////

    if (ldu.pot >= 4050 && ldu.brake == ON)
    {
        vcu.launchFlag = ON;
    }
    if (vcu.launchFlag == ON && ldu.pot < 1000)
    {
        vcu.launchFlag = OFF;
    }
}

void brakeHandler(void)
{
    if (vcu.state != burnout && iboost.pedal > 15)
    {
        canIOset(brake, ON);
    }
    else
    {
        canIOset(brake, OFF);
    }
}

uint8_t getResetCause(void)
{
    uint8_t reset_cause;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_cause = RESET_CAUSE_LOW_POWER_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        // This reset is induced by calling the ARM CMSIS
        // `NVIC_SystemReset()` function!
        reset_cause = RESET_CAUSE_SOFTWARE_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
    }
    // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
    // ensure first that the reset cause is NOT a POR/PDR reset. See note
    // below.
    /* else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
     {
         reset_cause = RESET_CAUSE_BROWNOUT_RESET;
     }*/
    else
    {
        reset_cause = RESET_CAUSE_UNKNOWN;
    }

    // Clear all the reset flags or else they will remain set during future
    // resets until system power is fully removed.
    //__HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_cause;
}

void taskCheck(void)
{
    uint32_t timeNow = HAL_GetTick();
    taskTime.TaskLoop_max = timeNow - taskTime.TaskLoop_lastRun;
    taskTime.Task10ms_max = timeNow - taskTime.Task10ms_lastRun;
    taskTime.Task100ms_max = timeNow - taskTime.Task100ms_lastRun;
    taskTime.Task250ms_max = timeNow - taskTime.Task250ms_lastRun;

}