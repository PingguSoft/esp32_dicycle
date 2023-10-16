#include "dicycle.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/


/*
*****************************************************************************************
* LEG Class
*****************************************************************************************
*/
float fixAngle(float angle) {
    if (angle < -180) {
        angle += 360;
    } else if (angle >= 180) {
        angle -= 360;
    }
    return angle;
}

float roundUp(float v) {
    return round(v * 100.0f) / 100.0f;
}

/*
*****************************************************************************************
* HexaPod Class
*****************************************************************************************
*/
Dicycle::Dicycle(float upperLegLen, float lowerLegLen) :
    _gyroProc(HW_SERVO_UPDATE_FREQ) {
    _lLastChcked  = 0;
    _iInterval    = 1000 / HW_SERVO_UPDATE_FREQ;    // default update interval (50Hz)
    _upperLegLen  = upperLegLen;
    _lowerLegLen  = lowerLegLen;
    _maxHeight    = _upperLegLen + _lowerLegLen;
    _const0       = (_upperLegLen * _upperLegLen) + (_lowerLegLen * _lowerLegLen);
    _const1       = 2 * _upperLegLen * _lowerLegLen;
    _midHeight    = 130;
}

void Dicycle::setup() {
    // _pHW = pHW;
    // _iInterval = 1000 / 100; //_pHW->getServoFreq();
    _gyroProc.setup();
    _actuator.setup();
}

void Dicycle::moveLeg(float x, float z) {

}

void Dicycle::update(unsigned long ts, bool isWalk, Vector vMov, Rotator rRot) {
    _lLastChcked = ts;
    int speed[2];

    _gyroProc.process(ts, rRot, speed);
}
