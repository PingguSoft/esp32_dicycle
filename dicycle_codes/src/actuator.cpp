#include <math.h>
#include "utils.h"
#include "actuator.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"


/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
const uint8_t _tbl_pins[2][2] = {
    {   PIN_L_PHASE_A, PIN_L_PHASE_B },
    {   PIN_R_PHASE_A, PIN_R_PHASE_B }
};

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/
typedef struct {
    uint8_t         unit;
    uint32_t        status;
    int16_t         count;
    unsigned long   timeStamp;
} pcnt_evt_t;

/*
*****************************************************************************************
* VARIABLES
*****************************************************************************************
*/
static const int16_t  _k_ctr_limit = 16384;
static const uint16_t _k_min_rc = 1000;
static const uint16_t _k_max_rc = 2000;

/*
*****************************************************************************************
* FUNCTIONS
*****************************************************************************************
*/

/*
*****************************************************************************************
* ISR
*****************************************************************************************
*/
void IRAM_ATTR isrHandlerPCNT(void* arg) {
    struct Actuator::wheel *pWheel = reinterpret_cast<struct Actuator::wheel*>(arg);
    uint32_t                status;

    pcnt_get_event_status((pcnt_unit_t)pWheel->unit, &status);

    if (status & PCNT_EVT_L_LIM) {
        pWheel->ctr_mult--;
    } else if (status & PCNT_EVT_H_LIM) {
        pWheel->ctr_mult++;
    }
}

void initPCNT(pcnt_unit_t unit, int gpio_a, int gpio_b = PCNT_PIN_NOT_USED,
        pcnt_channel_t channel = PCNT_CHANNEL_0, int16_t h_lim = 16384, int16_t l_lim = -16384) {

    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = gpio_a,           // set gpio for pulse input gpio
        .ctrl_gpio_num = gpio_b,            // set gpio for control, not used
        .lctrl_mode = PCNT_MODE_REVERSE,    // Rising A on LOW B  = CW Step
        .hctrl_mode = PCNT_MODE_KEEP,       // Rising A on HIGH B = CCW Step
        .pos_mode = PCNT_COUNT_INC,         // increment the counter on positive edge
        .neg_mode = PCNT_COUNT_DIS,         // do nothing on falling edge
        .counter_h_lim = h_lim,
        .counter_l_lim = l_lim,
        .unit = unit,                       // PCNT unit number
        .channel = PCNT_CHANNEL_0
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(unit, 250);
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_1, thres1);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_0, thres0);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);

    pcnt_counter_pause(unit);
    pcnt_intr_enable(unit);
    pcnt_counter_resume(unit);
    pcnt_counter_clear(unit);
}

Actuator::Actuator() {
    _pose = {0, 0, 0};
    _wheels[IDX_LWHEEL] = {(uint8_t)PCNT_UNIT_0, 0, 0, new Servo()};
    _wheels[IDX_RWHEEL] = {(uint8_t)PCNT_UNIT_1, 0, 0, new Servo()};
}

void Actuator::initESC(Servo *pESC, int pin) {
    pESC->attach(pin, _k_min_rc, _k_max_rc);
    pESC->setPeriodHertz(50);
    delay(2);

    // pESC->writeMicroseconds(MAX_PULSE_WIDTH);
    // delay(2);
    // pESC->writeMicroseconds(_k_min_rc);
    // delay(2);
    pESC->writeMicroseconds(DEFAULT_PULSE_WIDTH);
    delay(2);
}

uint8_t Actuator::getStatus(uint8_t idx) {
    return digitalRead(_tbl_pins[idx][1]) << 4 | digitalRead(_tbl_pins[idx][0]);
}

long Actuator::getCounter(uint8_t idx) {
    int16_t cnt;

    pcnt_get_counter_value((pcnt_unit_t)idx, &cnt);
    return _wheels[idx].ctr_mult * long(_k_ctr_limit) + cnt;
}

void Actuator::resetCounter(uint8_t idx) {
    _wheels[idx].ctr_mult = 0;
    _wheels[idx].last_tick = 0;
    pcnt_counter_pause((pcnt_unit_t)idx);
    pcnt_counter_clear((pcnt_unit_t)idx);
    pcnt_counter_resume((pcnt_unit_t)idx);
}

void Actuator::setup() {
    for (int j = 0; j < ARRAY_SIZE(_tbl_pins); j++)
        for (int i = 0; i < sizeof(_tbl_pins[0]); i++)
            pinMode(_tbl_pins[j][i], INPUT_PULLDOWN);

    initPCNT(PCNT_UNIT_0, PIN_L_PHASE_A, PIN_L_PHASE_B, PCNT_CHANNEL_0, _k_ctr_limit, -_k_ctr_limit);
    initPCNT(PCNT_UNIT_1, PIN_R_PHASE_A, PIN_R_PHASE_B, PCNT_CHANNEL_0, _k_ctr_limit, -_k_ctr_limit);
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, isrHandlerPCNT, (void*)&_wheels[IDX_LWHEEL]);
    pcnt_isr_handler_add(PCNT_UNIT_1, isrHandlerPCNT, (void*)&_wheels[IDX_RWHEEL]);

    initESC(_wheels[IDX_LWHEEL].pESC, PIN_L_PWM);
    initESC(_wheels[IDX_RWHEEL].pESC, PIN_R_PWM);
}

/*
const uint8_t _tblSpeed[27] = {
        0,  50,  60,  70,  80,  90, 100, 110, 115, 120,
    125, 130, 135, 140, 145, 150, 155, 160, 165, 170,
    175, 180, 185, 190, 195, 200, 205
};
uint8_t getSpeed(uint8_t spd) {
    uint8_t idx = spd / 10;
    uint8_t rem = spd % 10;
    float step  = (_tblSpeed[idx + 1] - _tblSpeed[idx]) / 10;

    uint8_t out = _tblSpeed[idx] + uint8_t(rem * step);

    out = map(out, 0, 255, 0, 100);
    return out;
}
 */

void Actuator::setMotor(int speedL, int speedR) {
    int     limAbsSpeed;

    limAbsSpeed = map(speedL, -255, 255, 2000, 1000);
    _wheels[IDX_LWHEEL].pESC->writeMicroseconds(limAbsSpeed);

    limAbsSpeed = map(speedR, -255, 255, 2000, 1000);
    _wheels[IDX_RWHEEL].pESC->writeMicroseconds(limAbsSpeed);
    // LOG("speed: %6d, %6d\n", speedL, speedR);
}

void Actuator::drive(int angle, int speed) {
    int speedL;
    int speedR;

    if (angle >= 0) {
        float rad = radians(angle);
        speedL = speed;
        speedR = speed * cos(rad);
    } else {
        float rad = radians(-angle);
        speedL = speed * cos(rad);
        speedR = speed;
    }
    //LOG("ang:%6d,  spd:%6d, %6d\n", angle, speedL, speedR);
    setMotor(speedL, speedR);
}

void Actuator::getDelta(long tick_diff_l, long tick_diff_r, float *dtheta, float *dist) {
    float w_circ = 2 * M_PI * WHEEL_RADIUS_MM;
    float dist_l = w_circ * tick_diff_l / TICKS_PER_CYCLE;
    float dist_r = w_circ * tick_diff_r / TICKS_PER_CYCLE;
    *dtheta = (dist_r - dist_l) / (2 * AXLE_HALF_WIDTH_MM);
    *dist   = (dist_r + dist_l) / 2;
}

void Actuator::getOdometry(odometry_t *pOdometry) {
    pOdometry->millis = millis();
    pOdometry->left   = getCounter(IDX_LWHEEL);
    pOdometry->right  = getCounter(IDX_RWHEEL);
}

void Actuator::getPose(pose_t *pPose) {
    odometry_t odo;

    getOdometry(&odo);

    long  tick_diff_r = odo.right - _wheels[IDX_RWHEEL].last_tick;
    long  tick_diff_l = odo.left  - _wheels[IDX_LWHEEL].last_tick;

    _wheels[IDX_RWHEEL].last_tick = odo.right;
    _wheels[IDX_LWHEEL].last_tick = odo.left;

    float dtheta, dist;
    getDelta(tick_diff_l, tick_diff_r, &dtheta, &dist);

    float dx  = dist * cos(_pose.theta);
    float dy  = dist * sin(_pose.theta);
    float pi2 = (2 * M_PI);

    // global coord information
    _pose.millis = millis();
    _pose.d_dist = dist;
    _pose.x     += dx;
    _pose.y     += dy;
    _pose.theta += dtheta;
    if (_pose.theta >= pi2)
        _pose.theta -= pi2;
    else if (_pose.theta <= -pi2)
        _pose.theta += pi2;

    // LOG("%8ld tick:%8ld,%8ld, diff_tick:%8ld,%8ld, dtheta:%6.2f, dist:%6.2f, delta_xy:%6.2f, %6.2f, pose:%8ld, %8ld, %6.2f\n",
    //     _pose.millis, tick_l, tick_r, tick_diff_l, tick_diff_r, dtheta, dist, dx, dy, _pose.x, _pose.y, _pose.theta);
    *pPose = _pose;
}

void Actuator::resetPose() {
    resetCounter(IDX_RWHEEL);
    resetCounter(IDX_LWHEEL);
    _pose = { 0, 0, 0};
}


void Actuator::calibrate(int key) {
    static int leg = 0;
    static long last_ctr = 0;
    static unsigned long last_ms = 0;
    unsigned long ts, diff;
    int us;
    long ctr;
    float rot;

    switch (key) {
        case '1':
        case '2':
            leg   = key - '1';
            break;

        case ',':
            us = _wheels[leg].pESC->readMicroseconds();
            us -= 5;
            _wheels[leg].pESC->writeMicroseconds(us);
            LOG("wheel:%d, us:%4d\n", leg, us);
            break;

        case '.':
            us = _wheels[leg].pESC->readMicroseconds();
            us += 5;
            _wheels[leg].pESC->writeMicroseconds(us);
            LOG("wheel:%d, us:%4d\n", leg, us);
            break;

        case 'r':
            resetCounter(IDX_LWHEEL);
            resetCounter(IDX_RWHEEL);
            // no break

        case '/':
            ts  = millis();
            ctr = getCounter(IDX_LWHEEL);

            diff = ts - last_ms;
            rot  = (ctr - last_ctr) / (34 * 360.0);
            rot  = rot * 60000 / diff;

            LOG("wheel_l, ctr:%8ld, rpm:%6.1f\n", ctr, rot);
            last_ctr = ctr;
            last_ms  = ts;

            ctr = getCounter(IDX_RWHEEL);
            LOG("wheel_r, ctr:%8ld, ang:%5d\n", ctr, ctr / 34);
            break;

        case 'e':
            _wheels[0].pESC->writeMicroseconds(_k_max_rc);
            _wheels[1].pESC->writeMicroseconds(_k_max_rc);
            LOG("max:%4d\n", _k_max_rc);
            break;

        case 'd':
            _wheels[0].pESC->writeMicroseconds(DEFAULT_PULSE_WIDTH);
            _wheels[1].pESC->writeMicroseconds(DEFAULT_PULSE_WIDTH);
            LOG("center:%4d\n", DEFAULT_PULSE_WIDTH);
            break;

        case 'c':
            _wheels[0].pESC->writeMicroseconds(_k_min_rc);
            _wheels[1].pESC->writeMicroseconds(_k_min_rc);
            LOG("min:%4d\n", _k_min_rc);
            break;
    }
}
