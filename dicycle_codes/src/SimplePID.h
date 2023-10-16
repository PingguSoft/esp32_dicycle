#ifndef __SIMPLE_PID__
#define __SIMPLE_PID__
#include <Arduino.h>

class SimplePID {
#define ITERM_MAX_ERROR         25      // Iterm windup constants for PI control //40
#define ITERM_MAX               8000    // 5000

public:
    SimplePID() {
        reset();
    }

    float getStabilityPD(float dt, float input, float setPoint, float p, float d) {
        float error;
        float output;

        error = setPoint - input;
        // Kd is implemented in two parts
        //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
        //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
        output = p * error + (d * (setPoint - _setOld) - d * (input - _errOld2)) / dt;
        _errOld2 = _errOld;
        _errOld = input;  // error for Kd is only the input component
        _setOld = setPoint;
        return (output);
    }

    float getSpeedPI(float dt, float input, float setPoint, float p, float i) {
        float error;
        float output;

        error = setPoint - input;
        _errSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
        _errSum = constrain(_errSum, -ITERM_MAX, ITERM_MAX);

        output = p * error + i * _errSum * dt * 0.001; // dt is in miliseconds...
        return (output);
    }

    void reset() {
        _errSum = 0;
        _errOld = 0;
        _errOld2 = 0;
        _setOld = 0;
    }

private:
    float       _errSum;
    float       _errOld;
    float       _errOld2;
    float       _setOld;
};
#endif