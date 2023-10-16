#ifndef _MABEL_H_
#define _MABEL_H_

#include "config.h"
#include "VecRot.h"
#include "GyroProc.h"
#include "actuator.h"

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

class Dicycle {
public:
    Dicycle(float upperLegLen, float lowerLegLen);
    void setup();
    void moveLeg(float x, float z);
    void update(unsigned long ts, bool isWalk, Vector vMov, Rotator rRot);
    GyroProc *getGyroProc() { return &_gyroProc; }
    Actuator *getHW()       { return &_actuator; }

private:
    Vector    _vBodyPos;
    Actuator  _actuator;
    int       _iInterval;         // (unit:ms)
    unsigned long _lLastChcked;
    float     _upperLegLen;
    float     _lowerLegLen;
    float     _maxHeight;
    float     _const0;
    float     _const1;
    float     _midHeight;

    int       _iYawRot;
    GyroProc  _gyroProc;
};

#endif