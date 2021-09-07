#if !defined(__ENCODER__H__)
#define __ENCODER__H__

#include "can_setup.h"
#include "vcu.h"

//#include "stm32f1xx_hal.h"

//event ID 
#define EVENT 0x18FF0FF2   //canID swipe tap or turn
#define DATA 0x18FF11F2    //canID encoder transmit data
#define CONTROL 0x18EFF221 //canID recieve command


//screen ID  Project specific
#define HOME 0x01 

#define FWD_CFM 0x02
#define FWD_REQ 0X03 

#define NTRL_REQ 0x04
#define NTRL_CFM 0x05 

#define REV_CFM 0x06
#define REV_REQ 0X07

#define CHARGE_STATS 0X08
#define CS_TEMP 0x01
#define CS_SOC 0x04
#define CS_AMP 0x08

#define WINDOWS 0X09

#define BATTERY 0X0A
#define BATT_VOLT 0x01
#define BATT_TEMP 0x04

#define LAUNCH  0x0B



//SWIPE DEFINES
#define SWIPE_DOWN 0x82
#define SWIPE_UP 0x81
#define SWIPE_LEFT 0x83
#define SWIPE_RIGHT 0x84
#define SWIPE_NONE 0x80


//ACTIVE VALUE DEFINES  
#define VAL_0 0x00  
#define VAL_1 0X01
#define VAL_2 0X02
#define VAL_3 0X04
#define VAL_4 0X08
#define VAL_5 0X10
#define VAL_6 0X20
#define VAL_7 0X40
#define VAL_8 0X80

//TAP ZONE DEFINES
#define ZONE_1 0X8001
#define ZONE_2 0X8002
#define ZONE_3 0X8004
#define ZONE_4 0X8008
#define ZONE_5 0X8010
#define ZONE_6 0X8020
#define ZONE_7 0X8040
#define ZONE_8 0X8080
#define ZONE_9 0X8100
#define ZONE_10 0X8200
#define ZONE_11 0X8400
#define ZONE_12 0X8800
#define ZONE_13 0X9000
#define ZONE_14 0XA000
#define ZONE_15 0XC000  

typedef struct
{
    // Set Values
    uint8_t currentScreen; // 0x00 Screen 0, 0x01 Screen 1, etc
    uint8_t knob;          // = 0x00;
    uint16_t tap;          // = 0x00;
    uint8_t swipe;         // = 0x00;
    uint8_t valueId;       // 0x00 no values, 0x01 value 1, etc
    uint16_t currentVal;
    uint8_t dispCode;
    uint8_t activeId;
    uint8_t state;
} encoder_t;
encoder_t te;

typedef enum te_state
{
    sleep,
    wake,
    waitInput,
    startup,
    gearslct,
    chargeing,
    launch,
    pwrStats,
} te_state;

//uint8_t te.state;
uint8_t setPrevState;

void encoderHandler(void);
void setWidget(uint8_t screenNum, uint8_t valueId, uint16_t currentVal);
void resetWidget(uint8_t screenNum, uint8_t valueId, uint16_t currentVal);
void setBacklight(uint8_t level);
void getEvent(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void getData(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);

#endif // __ENCODER__H__
