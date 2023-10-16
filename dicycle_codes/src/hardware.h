#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#define  ENABLE_DEBUG_OUTPUT
#include <ESP32Servo.h>
#include "config.h"
#include "VecRot.h"
#include "utils.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/

/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/
class Hardware {
    struct _motor_cfg {
        uint8_t     pin_pha_a;
        uint8_t     pin_pha_b;
        uint8_t     pin_servo;
        uint8_t     inv;
        int         w_speed;
        int         uS;
        int         r_speed;
        Servo       *servo;
    };

public:
    Hardware();
    void setup(void);
    void setSpeed(int leg, int speed);
    void calibrate(int key);
    void dump();
    int  getServoFreq()                            { return HW_SERVO_UPDATE_FREQ; }
    int  checkBattery();
    void step(void);


private:
    void loadConfig(void);
    void saveConfig(void);
    uint16_t analogReadAvg(uint16_t pin);

    int                             _tmrCtr;
    static struct _motor_cfg        _cfgMotors[BODY_NUM_LEGS];

};

#endif