#include "tasks.h"
#include "main.h"
#include "can_setup.h"
#include "gauges.h"
#include "vcu.h"
#include "test.h"
#include "water_pump.h"
#include "dcdc.h"
#include "tasks.h"

extern TIM_HandleTypeDef htim6;
extern IWDG_HandleTypeDef hiwdg;

#define G_MAX_MS_COUNTER 4294967296

uint32_t G_mSCounter = 0;
uint8_t G_10mStask = 0;
uint8_t G_100mStask = 0;
uint8_t G_250mStask = 0;
uint8_t G_500mStask = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    /* htim17 1ms */
    if (&htim6 == htim)
    {

        static uint16_t cpt2 = 0;

        G_mSCounter++;

        if (G_mSCounter % 500 == 0)
        {
            G_500mStask = 1;
        }

        if (G_mSCounter >= G_MAX_MS_COUNTER)
        {
            G_mSCounter = 0;
        }

        cpt2++;

        /* 1 S, reinitiate cpt2 */
        if (cpt2 >= 1000)
        {
            cpt2 = 0;
        }

        /* 100 mS */
        if (cpt2 % 10 == 0)
        {
            G_10mStask = 1;
        }

        if (cpt2 % 100 == 0)
        {
            G_100mStask = 1;
        }

        if (cpt2 % 250 == 0)
        {

            G_250mStask = 1;
        }
    }
}

void taskHandler(void)
{
    if (G_10mStask)
    {
        vcuState();
        throttleHandler();
        canIOsend();
        regenHandler();
        brakeHandler();
        dcdcHandler(vcu.state);
        G_10mStask = 0;
    }

    if (G_100mStask)
    {
        updateSpeed(ldu.rpm);
        updateTach(ldu.amps);
        ioHandler();
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        G_100mStask = 0;
    }

    if (G_250mStask)
    {
        updateSOC(BMS[0].chargeState, ldu.amps);
        updateTemp(ldu.hsTemp);
        vehicleComms();
        encoderHandler();
        wpHandler();
        G_250mStask = 0;
    }

    if (G_500mStask)
    {
        //HAL_IWDG_Refresh(&hiwdg);
        
        G_500mStask = 0;
    }
}
