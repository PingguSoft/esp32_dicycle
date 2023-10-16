#include <FS.h>
#include <SPIFFS.h>
#include "GyroProc.h"

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


GyroProc::GyroProc(int hz) :
    _pidAngle("Angle PID", false, 0.07, 0.04, 0, 8000),
    _pidOut("Output PID", false, 0.5, 0.0, 17),

    _pidRoll("Roll  PID", true, 0.95, 0.35, 0)
{
    _pidAngle.setTarget(0.0f);
    _pidRoll.setTarget(0.0f);
    _iSel = 0;
    _iInterval = 1000 / hz;
    _pPID = &_pidAngle;
    _p = _pPID->getP();
    _i = _pPID->getI();
    _d = _pPID->getD();
    _m = 60;
    _eff = 80;

    _estSpeed = 0;
    _output   = 0;
    _motor_speed[0] = 0.0f;
    _motor_speed[1] = 0.0f;
    _isStand = false;
}

void GyroProc::setup() {
    memset(_offsets, 0, sizeof(_offsets));

    LOG("gyro setup !!!\n");
    File file = SPIFFS.open(FILE_GYRO_CFG);
    LOG("1\n");
    if (file) {
        if (file.read((uint8_t*)_offsets, sizeof(_offsets)) > 0) {
            LOG("gyro cal data loaded !!!\n");
        }
        file.close();
        LOG("2\n");
        _gyro.setup(_offsets);
        LOG("3\n");
    } else {
        LOG("gyrocal.dat not found, start calibration !!\n");
        calibrate();
    }
    LOG("gyro setup done !!!\n");
}

bool GyroProc::calibrate(void) {
    bool ret;

    File file = SPIFFS.open(FILE_GYRO_CFG, FILE_WRITE);
    if (!file) {
        LOG("file write open error\n");
        return false;
    }
    ret = _gyro.calibrate(_offsets);
    file.write((uint8_t*)_offsets, sizeof(_offsets));
    file.close();
    _gyro.setup(_offsets);

    // LOG("cal offsets\n");
    // for (int i = 0; i < ARRAY_SIZE(_offsets); i++) {
    //     LOG("%6d, ", _offsets[i]);
    // }
    // LOG("\n");

    return ret;
}

float GyroProc::fixAngle(float fTgt, float fSrc) {
    if (abs(fTgt - fSrc) > 180) {
        if (fSrc > 0) {
            fTgt += 360;
        } else if (fSrc < 0) {
            fTgt -= 360;
        }
    }
    return fTgt;
}

bool GyroProc::isValid(float angle) {
    return !isnan(angle) && (-180 <= angle && angle <= 180);
}

void GyroProc::setTarget(float roll, float pit) {
    _pidRoll.setTarget(roll);
    _pidAngle.setTarget(pit);
}


#define MAX_ACCEL               10       // Maximun motor acceleration (MAX RECOMMENDED VALUE: 8) (default:7)
float getMotorSpeed(float prevSpeed, float nowSpeed) {
    float speed = prevSpeed;

    if ((speed - nowSpeed) > MAX_ACCEL)
        speed -= MAX_ACCEL;
    else if ((speed - nowSpeed) < -MAX_ACCEL)
        speed += MAX_ACCEL;
    else
        speed = nowSpeed;

    return speed;
}


Rotator GyroProc::process(unsigned long ts, Rotator rot, int output[2]) {
    static Rotator ret;
    static int     ctr = 0;
    static ypr_t   old_ypr;
    static unsigned long old_ts;
    ypr_t           ypr;

    if (!_gyro.loop())
        return ret;


    ypr = _gyro.getYPR();
    if (!isValid(ypr.yaw) || !isValid(ypr.pitch) || !isValid(ypr.roll)) {
        return ret;
    }
    //  gyro sensor output
    //  yaw (left:-, right:+)
    //
    //           pitch-
    //             |
    //  -roll   ---+---  roll+
    //             |
    //           pitch+
    //

#if 0
    float ap = abs(ypr.pitch);

    if (!_isStand && ap < 3) {
        reset();
        old_ypr  = ypr;
        old_ts   = ts;
        ctr      = 0;

        _output   = 0;
        _estSpeed = 0;
        _motor_speed[0] = 0;
        _motor_speed[1] = 0;

        _isStand = true;
    } else if (_isStand && ap > 60) {
        _isStand = false;
    }

    if (_isStand) {
        float oldSpeed = (_motor_speed[0] + _motor_speed[1]) / 2; // Positive: forward

        float angularV = (ypr.pitch - old_ypr.pitch) * _eff;    // 90 is an empirical extracted factor to adjust for real units
        float estSpeed = -oldSpeed - angularV;                 // We use robot_speed(t-1) or (t-2) to compensate the delay
        _estSpeed = (_estSpeed * 0.95) + (estSpeed * 0.05);     // low pass filter on estimated speed

        float fDelta = ts - old_ts;
        if (fDelta >= 20 || fDelta <= 0) {
            fDelta = 20;
        }
        // SPEED CONTROL: This is a PI controller.
        //    input:user throttle, variable: estimated robot speed, output: target robot angle to get the desired speed
        float targetAngle = _pid.getSpeedPI(fDelta, _estSpeed, rot.pitch, _pidAngle.getP(), _pidAngle.getI());
        targetAngle = constrain(targetAngle, -30, 30); // limited output

        // Stability control: This is a PD controller.
        //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
        //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
        float out = _pid.getStabilityPD(fDelta, ypr.pitch, targetAngle, _pidOut.getP(), _pidOut.getD());
        _output += out;
        _output = constrain(_output, -BODY_MAX_PITCH_OUT, BODY_MAX_PITCH_OUT); // Limit max output from control

        float speed[2];
        speed[0] = _output + rot.roll;
        speed[1] = _output - rot.roll;
        speed[0]  = constrain(speed[0], -BODY_MAX_PITCH_OUT, BODY_MAX_PITCH_OUT);
        speed[1]  = constrain(speed[1], -BODY_MAX_PITCH_OUT, BODY_MAX_PITCH_OUT);
        _motor_speed[0] = getMotorSpeed(_motor_speed[0], speed[0]);
        _motor_speed[1] = getMotorSpeed(_motor_speed[1], speed[1]);
        output[0] = _motor_speed[0] * _m;
        output[1] = _motor_speed[1] * _m;

        if (ctr == 0) {
            LOG("P:%7.2f => %7.2f, aV:%7.2f, oV:%7.2f, estSpeed:%7.2f, _estSpeed:%7.2f, tA:%7.2f, out:%7.2f, _output:%7.2f, spd:%6d, %6d ",
                old_ypr.pitch, ypr.pitch, angularV, oldSpeed, estSpeed, _estSpeed, targetAngle, out, _output, output[0], output[1]);
            LOG("%s P:%7.2f, I:%7.2f, D:%7.2f, M:%7.2f, E:%7.2f\n", _pPID->getName().c_str(), _pPID->getP(), _pPID->getI(), _pPID->getD(), _m, _eff);
        }
        old_ts  = ts;
        old_ypr = ypr;
        ctr = (ctr + 1) % 10;
    } else {
        output[0] = 0;
        output[1] = 0;
    }
#else
    // float out = _pidAngle.compute(ts, ypr.pitch, 30);
    // output[0] = out * _m;
    // output[1] = out * _m;

    // if (ctr == 0) {
    //     LOG("Y:%7.2f, P:%7.2f, R:%7.2f out:%7.2f, spd:%7.2f\n", ypr.yaw, ypr.pitch, ypr.roll, out, output[0]);
    // }

    LOG("Y:%7.2f, P:%7.2f, R:%7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);
#endif

/*
    ret.yaw   = rot.yaw;
    ret.roll  = _pidRoll.compute(ts, ypr.roll, BODY_MAX_ROLL_OUT);
    if (ctr == 0) {
        LOG("%10ld Y:%7.2f, P:%7.2f, R:%7.2f OUT_PR(%7.2f, %7.2f)\n", ts, ypr.yaw, ypr.pitch, ypr.roll,
            ret.pitch, ret.roll);
    }
*/

    return ret;
}

void GyroProc::togglePID() {
    _iSel = (_iSel + 1) % 2;
    _pPID = (_iSel == 0) ? &_pidAngle : &_pidOut;
    _p = _pPID->getP();
    _i = _pPID->getI();
    _d = _pPID->getD();
    LOG("%s P:%7.2f, I:%7.2f, D:%7.2f\n", _pPID->getName().c_str(), _p, _i, _d);
}

void GyroProc::setPID() {
    _pPID->set(_p, _i, _d);
    LOG("%s P:%7.2f, I:%7.2f, D:%7.2f, M:%7.2f, E:%7.2f\n", _pPID->getName().c_str(), _pPID->getP(), _pPID->getI(), _pPID->getD(), _m, _eff);
}

void GyroProc::incP() {
    _p += 0.02f;
    setPID();
}

void GyroProc::decP() {
    _p -= 0.02f;
    setPID();
}

void GyroProc::incI() {
    _i += 0.02f;
    setPID();
}

void GyroProc::decI() {
    _i -= 0.02f;
    setPID();
}

void GyroProc::incD() {
    _d += 0.02f;
    setPID();
}

void GyroProc::decD() {
    _d -= 0.02f;
    setPID();
}

void GyroProc::reset() {
    _pidOut.reset();
    _pidAngle.reset();
    _pidRoll.reset();

    _pid.reset();
}
