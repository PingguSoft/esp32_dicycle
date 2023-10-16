#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_
#include <ESP32Servo.h>
#include "config.h"


/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
enum {
    IDX_LWHEEL = 0,
    IDX_RWHEEL
};

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/
typedef struct {
    unsigned long   millis;     // ms
    long            x;          // mm
    long            y;          // mm
    uint16_t        d_dist;     // mm
    float           theta;      // radian
} __attribute__((packed)) pose_t;

typedef struct {
    unsigned long   millis;     // ms
    long            left;       // ticks
    long            right;      // ticks
} __attribute__((packed)) odometry_t;

/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/

class Actuator {
public:
    Actuator();

    void    setup();
    int     process();
    void    setMotor(int speedL, int speedR);
    void    drive(int angle, int speed);
    void    getOdometry(odometry_t *pOdometry);
    void    getPose(pose_t* pPose);
    void    resetPose();
    void    calibrate(int key);
    uint8_t getStatus(uint8_t idx);
    friend void isrHandlerPCNT(void *arg);

private:
    typedef enum {
        DIR_FW = 0,
        DIR_REV = 1,
    } dir_t;

    struct wheel {
        uint8_t     unit;
        long        ctr_mult;
        long        last_tick;
        Servo      *pESC;
    };

    struct wheel    _wheels[2];
    pose_t          _pose;

    void    initESC(Servo *pESC, int pin);
    long    getCounter(uint8_t idx);
    void    resetCounter(uint8_t idx);

    void    getDelta(long tick_diff_l, long tick_diff_r, float *dtheta, float *dist);
};

#endif
