#ifndef _GYRO_PROC_H_
#define _GYRO_PROC_H_

#include "config.h"
#include "VecRot.h"
#include "GyroDev.h"
#include "PIDController.h"
#include "SimplePID.h"
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

class GyroProc {
public:
    GyroProc(int hz);

    void    setup();
    bool    calibrate(void);
    bool    isValid(float angle);
    Rotator process(unsigned long ts, Rotator rot, int output[2]);
    void    togglePID();
    void    incP();
    void    decP();
    void    incI();
    void    decI();
    void    incD();
    void    decD();
    void    reset();
    void    setTarget(float roll, float pit);
    void    incM() { _m++; setPID(); }
    void    decM() { _m--; setPID(); }
    void    incE() { _eff++; setPID(); }
    void    decE() { _eff--; setPID(); }

private:
    void  setPID();
    float fixAngle(float fTgt, float fSrc);

    GyroDev         _gyro;

    PIDController   _pidAngle;
    PIDController   _pidOut;
    PIDController   _pidRoll;
    PIDController  *_pPID;

    SimplePID       _pid;

    bool            _isStand;
    ypr_t           _oldYpr;
    float           _motor_speed[2];
    float           _estSpeed;
    float           _output;
    float           _eff;

    Rotator         _rotTgt;

    int             _iInterval;
    float           _p, _i, _d, _m;
    int             _iSel;
    int16_t         _offsets[6];
};

#endif