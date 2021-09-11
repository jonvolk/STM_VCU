#if !defined(__VCU_H_)
#define __VCU_H_

#include "stm32f105xc.h"
#include "can_setup.h"
#include "stdbool.h"
#include "dcdc.h"
#include "my_math.h"

/*
Bit 0: key
Bit 1: motor
Bit 2: dir engagaed
Bit 3: in motion
Bit 4: tender
Bit 5: 
Bit 6:
Bit 7:
*/

/*
Bit 0: cruise
Bit 1: start
Bit 2: brake
Bit 3: forward
Bit 4: reverse
Bit 5: bms
*/

//LDU Parameter Index Defines
#define BOOST 0
#define FWEAK 1
#define FSLIP_MIN 5
#define FSLIP_MAX 6
#define THROTMAX 34
#define THROTRAMP 51
#define BRAKE_NOM_PEDAL 55
#define BRAKE_PEDAL_RAMP 56
#define IDLE_THROT_LIM 63
#define IDLE_MODE 64

//LDU Direction
#define FWD 255
#define NTRL 2
#define REV 3

//DIO Defines
#define NUETRAL 0X00
#define FORWARD 0X10
#define REVERSE 0X08

//CHARGER ENABLED
#define CHRG_ENABLED 5

#define ON 1
#define OFF 0

uint32_t ADC_data[3];
uint32_t tempdata;

typedef enum adc
{
    lvread,
    throttle,
    yaw,
} adc;

typedef struct LDU_t
{
    volatile uint16_t rpm;
    volatile uint16_t mTemp;
    volatile uint16_t hsTemp;
    volatile uint16_t potNom;
    volatile uint8_t dir;
    volatile uint8_t mode;
    volatile uint16_t amps;
    volatile uint8_t brake;
    volatile uint16_t pot;
    volatile uint16_t pot2;

} LDU_t;
LDU_t ldu;

typedef struct iBooster_t
{
    volatile uint16_t pedal;

} iBooster_t;
iBooster_t iboost;

typedef struct vcu_t
{
    volatile uint8_t dio;
    //volatile uint8_t gear;
    //volatile uint8_t mode;
    volatile uint8_t key;
    volatile uint8_t charge;
    volatile uint8_t chargeReq;
    //volatile uint16_t hp;
    //volatile uint32_t sprint60;
    volatile uint8_t state;
    volatile uint8_t launchFlag;
    volatile uint8_t burnFlag;

} vcu_t;
vcu_t vcu;

typedef struct charger_t
{
    uint8_t current;
    uint8_t chargerStatus;
    /* data */
} charger_t;
charger_t charger;

typedef struct bms_t
{
    uint16_t volt;
    uint8_t temp;
    uint8_t highCell;
    uint8_t lowCell;
    uint8_t cellDelta;
    uint8_t soc;
    uint8_t chargeRequest;
    uint8_t chargeState;
} bms_t;
bms_t BMS[2];

typedef enum ioBits
{
    cruise,
    start,
    brake,
    reverse, // drive unit Reverse
    forward, // drive unit Forward
    bms,

} ioBits;

typedef enum vcuStates
{
    off,
    on, //key on, inverter off
    charge_keyOff,
    charge_keyOn, //who even does that
    idle,         //key on, inverter on
    run,          //key on direction selected
    launchMode,   //break shit
    burnout,      //destroy tires
} vcuStates;

void vcuInit(void);
void decodeCAN(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);
void canIOset(int bit, int val);
void canIOsend(void);
void vcuState(void);
void brakeHandler(void);
void ioHandler(void);
void regenHandler(void);
void throttleHandler(void);
void canSet(uint8_t index, uint32_t value, uint8_t gain);
//void batteryTender(void);

#endif // __VCU_H_
