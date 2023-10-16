#include <Wire.h>
#include <FS.h>
#include <SPIFFS.h>
#include "hardware.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
static const int kSERVO_PERIOD_MIN   = 1000;
static const int kSERVO_PERIOD_MAX   = 2000;
static const int kDEG10_MIN          = 0;
static const int kDEG10_MAX          = 1800;

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************


/*
*****************************************************************************************
* VARIABLES
*****************************************************************************************
*/
struct Hardware::_motor_cfg Hardware::_cfgMotors[BODY_NUM_LEGS] = {
    // phase                                 , inv
    { PIN_L_PHASE_A, PIN_L_PHASE_B, PIN_L_PWM, 0, },
    { PIN_R_PHASE_A, PIN_R_PHASE_B, PIN_R_PWM, 1, }
};


/*
*****************************************************************************************
* FUNCTIONS
*****************************************************************************************
*/
Hardware::Hardware() {

}

void Hardware::setup(void) {
    loadConfig();

    for (int j = 0; j < BODY_NUM_LEGS; j++) {
        pinMode(_cfgMotors[j].pin_pha_a, INPUT);
        pinMode(_cfgMotors[j].pin_pha_b, INPUT);
        _cfgMotors[j].servo = new Servo();
        _cfgMotors[j].servo->attach(_cfgMotors[j].pin_servo, kSERVO_PERIOD_MIN, kSERVO_PERIOD_MAX);
        _cfgMotors[j].servo->setPeriodHertz(50);
        _cfgMotors[j].servo->writeMicroseconds(1500);
    }

    // for (int i = 0; i < BODY_NUM_LEGS; i++) {
    //     setLeg(i, 0, 0);
    // }
}

void Hardware::loadConfig(void) {
    // File file = SPIFFS.open(FILE_SERVO_CFG);
    // if (!file) {
    //     LOG("file not found\n");
    //     return;
    // }
    // file.read((uint8_t*)&_calAngles, sizeof(_calAngles));
    // file.close();
}

void Hardware::saveConfig(void) {
    // File file = SPIFFS.open(FILE_SERVO_CFG, FILE_WRITE);
    // if (!file) {
    //     LOG("file not found\n");
    //     return;
    // }
    // file.write((uint8_t*)&_calAngles, sizeof(_calAngles));
    // file.close();
}

/*
*****************************************************************************************
* FUNCTIONS
*****************************************************************************************
*/
void Hardware::dump() {
    LOG("---------- leg dump ----------\n");
    for (int i = 0; i < BODY_NUM_LEGS; i++) {
        LOG("leg:%d ", i + 1);
        LOG("\n");
    }
    LOG("------------------------------\n");
}

void Hardware::setSpeed(int leg, int speed) {
    int min_a = -100;
    int max_a =  100;

    int uS = (_cfgMotors[leg].inv == 0) ?
            map(speed, min_a, max_a, kSERVO_PERIOD_MIN, kSERVO_PERIOD_MAX) :
            map(speed, min_a, max_a, kSERVO_PERIOD_MAX, kSERVO_PERIOD_MIN);

    _cfgMotors[leg].w_speed = speed;
    _cfgMotors[leg].uS      = uS;
    _cfgMotors[leg].servo->writeMicroseconds(uS);
    LOG("leg:%d, speed:%5d, uS:%5d\n", leg, speed, uS);
}

void Hardware::calibrate(int key) {
    static int leg   = 0;

    switch (key) {
        case '1':
        case '2':
            leg   = key - '1';
            break;

        case ';':
            _cfgMotors[leg].w_speed -= 5;
            setSpeed(leg, _cfgMotors[leg].w_speed);
            break;

        case '\'':
            _cfgMotors[leg].w_speed += 5;
            setSpeed(leg, _cfgMotors[leg].w_speed);
            break;
    }


}

#define ANALOG_RETRY_COUNT      5

uint16_t Hardware::analogReadAvg(uint16_t pin) {
    uint16_t v;
    uint32_t sum = 0;
    uint16_t values[ANALOG_RETRY_COUNT];

    for (uint8_t i = 0; i < ANALOG_RETRY_COUNT; i++) {
        v = analogRead(pin);
        values[i] = v;
        sum += v;
    }

    uint16_t avg = sum / ANALOG_RETRY_COUNT;
    uint8_t  cnt = ANALOG_RETRY_COUNT;
    for (uint8_t i = 0; i < ANALOG_RETRY_COUNT; i++) {
        v = values[i];
        if (abs(v - avg) > 100) {
            sum -= v;
            cnt --;
        }
    }
    return (cnt > 0) ? (sum / cnt) : 0;
}

/*
    R1 = 44000, R2 = 22000
    Vadc = Vbatt * (R2 / R1 + R2)
    -----------------------------
    adc         Vbatt      level
    -----------------------------
    280  under: no battery  -1
    3330 above: 8.4V
    3230      : 8.2V         4
    3130      : 8.0V
    3030      : 7.8V
    2930      : 7.6V         3
    2870      : 7.4V
    2780      : 7.2V
    2700      : 7.0V         2
    2610      : 6.8V
    2530      : 6.6V
    2450      : 6.4V         1
        under :              0
    -----------------------------
*/

int Hardware::checkBattery() {
    int      level;
    uint16_t adc = analogReadAvg(PIN_PWR_ADC);

    //LOG("BATT:%5d\n", adc);
    if (adc > 3230) {
        level = 4;
    } else if (adc > 2930) {
        level = 3;
    } else if (adc > 2700) {
        level = 2;
    } else if (adc > 2450) {
        level = 1;
    } else if (adc > 2000) {
        level = 0;
    } else {
        level = -1;
    }
    return level;
}
