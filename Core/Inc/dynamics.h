#if !defined(__DYNAMICS_H_)
#define __DYNAMICS_H_

#include "can_setup.h"
#include "distance.h"

#define  GPS_POS
#define  GPS_TIME  


typedef struct 
{
    float_t lat; //Degree North positive, +90.00 to -90.00
    float_t lon; //Degrees +180.00 East to -180.00 West
    uint16_t speed; // .01 mph/bit
    uint16_t alt; // 1 ft/bit
    uint16_t course; // .01 deg/bit
    uint8_t sats; // number of satellites in use
    uint8_t valid_10hz; // 0=N/G 1=OK

}gps_t;
gps_t GPS;

typedef struct 
{
    uint8_t valid_5hz; // 0=N/G 1=OK
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;   
}utc_t;
utc_t UTC;

typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;
}accel_t;
accel_t ACCEL;

typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;
}yaw_t;
yaw_t YAW;












typedef struct 
{


}gyro_t;


int zero2sixty(void);
int getPitch(void);
void getDynamics(CAN_RxHeaderTypeDef *rxMsg, uint8_t *canRx);


#endif // __DYNAMICS_H_
