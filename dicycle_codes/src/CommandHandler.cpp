#include <FS.h>
#include <SPIFFS.h>
#include "CommandHandler.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/

//   controller orientation
//   yaw (left:-, right:+)
//             HEAD
//            pitch+
//              |
//    -roll  ---+---  roll+
//              |
//            pitch-
//


/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/


CommandHandler::CommandHandler(CommInterface* comm) :
    _pComm(comm),
    _robot(BODY_UPPER_LEG_LEN, BODY_LOWER_LEG_LEN)
{
    _nGait       = 0;
    _nLastBtnSts = 0;
    _isWalk      = false;
    _isFlash     = false;
    _isBalance   = false;
    memset(_isConnected, 0, sizeof(_isConnected));

    _vecMove.set(0, 0, 0);
    _rotMove.set(0, 0, 0);
    _legHeight   = 0;
}

CommandHandler::~CommandHandler() {
}

void CommandHandler::setup(void) {
    _robot.setup();

    loadUserData();
    // _statusLed.setup();
    // updateStatusLeds();
    _timedMove.setup(&_vecMove, &_rotMove);
    //_timedMove.go({ 0, 0, -120 }, { 0, 0, 0 }, 1000);
}

void CommandHandler::setup(CommInterface* comm) {
    _pComm = comm;
    setup();
}

void CommandHandler::loop() {
    static unsigned long lastTS = 0;
    unsigned long ts = millis();
    Rotator r;

    const float fPrevGain = MOTION_SMOOTH_GAIN;
    const float fCurGain  = 1.0f - MOTION_SMOOTH_GAIN;
    bool isConnected      = _isConnected[0] | _isConnected[1];

//    if (isConnected) {
        if (IS_ELAPSED(ts, lastTS, 20)) {
            float my = Utils::mapf(_rc.yaw,      1000, 2000, -100, 100);
            float mt = Utils::mapf(_rc.throttle, 1000, 2000, -100, 100);

            _vecMove.x     = 0;
            //_vecMove.y     = (my * fCurGain) + (_vecMove.y * fPrevGain);
            //_vecMove.z     = (mt * fCurGain) + (_vecMove.z * fPrevGain);

            float mr = Utils::mapf(_rc.roll,     1000, 2000, -40,   40);
            float mp = Utils::mapf(_rc.pitch,    1000, 2000, 100, -100);        // invert pitch

            _rotMove.yaw   = 0;
            _rotMove.pitch = (mp * fCurGain) + (_rotMove.pitch * fPrevGain);
            _rotMove.roll  = (mr * fCurGain) + (_rotMove.roll  * fPrevGain);

            lastTS = ts;
        }
//    }

    if (!_vecMove.equals(_vecOldMove)) {
        //_vecMove.dump("DIR");
        _vecOldMove.set(_vecMove);
    }

    if (!_rotMove.equals(_rotOldMove)) {
        //_rotMove.dump("ROT");
        _rotOldMove.set(_rotMove);
    }
    r = _rotMove;

    _timedMove.handle(ts);
    _robot.update(ts, _isWalk, _vecMove, r);
    // updateStatusLeds();
    // _statusLed.loop(ts);
}

bool CommandHandler::isSet(int shift, int state, int checkBtn) {
    int shiftMask  = checkBtn & (BTN_SHIFT_L | BTN_SHIFT_R);

    if (shiftMask != shift)
        return false;

    state &= ~(BTN_SHIFT_L | BTN_SHIFT_R);    // clear shift mask
    return bool(state & checkBtn);
}

void CommandHandler::setSwitches(int switches) {
    static int lastSwitches = 0;

    int  toggled   = (switches ^ lastSwitches);
    int  shift     = switches & (BTN_SHIFT_L | BTN_SHIFT_R);

    if (toggled == 0)
        return;

    lastSwitches = switches;
    if (isSet(shift, toggled, BTN_FLASH)) {
        _isFlash = (switches & BTN_FLASH);
        LOG("flash:%d\n", int(_isFlash));
    }

    // if (isSet(shift, toggled, BTN_GAIT)) {
    //     _nGait = (_nGait + 1) % _gaitMan.getGaitCnt();
    //     _pGait = _gaitMan.get(_nGait);
    //     _robot.setGait(_pGait);
    //     LOG("gait : %s\n", _pGait->getName().c_str());
    // }

    if (isSet(shift, toggled, BTN_WALK)) {
        _isWalk = (switches & BTN_WALK);
        LOG("waling : %d\n", int(_isWalk));
    }

    if (isSet(shift, toggled, BTN_HEIGHT_DEC)) {
        if (_vecMove.z >= (MIN_Z + 5)) {
            _vecMove.z -= 5;
            _vecMove.dump("DIR");
        }
    }

    if (isSet(shift, toggled, BTN_HEIGHT_INC)) {
        if (_vecMove.z <= (MAX_Z - 5)) {
            _vecMove.z += 5;
            _vecMove.dump("DIR");
        }
    }

    // if (isSet(shift, toggled, BTN_STEP_Z_DEC)) {
    //     if (_pGait && _pGait->getStep()->z > 0) {
    //         _pGait->getStep()->z--;
    //         _pGait->getStep()->dump("STEP");
    //     }
    // }

    // if (isSet(shift, toggled, BTN_STEP_Z_INC)) {
    //     if (_pGait && _pGait->getStep()->z < 50) {
    //         _pGait->getStep()->z++;
    //         _pGait->getStep()->dump("STEP");
    //     }
    // }

    // if (isSet(shift, toggled, BTN_STEP_INC)) {
    //     if (_pGait) {
    //         float steps = _pGait->getStepsPerSec() + 0.5f;
    //         if (steps < 100) {
    //             _pGait->setStepsPerSec(steps);
    //             LOG("steps per sec : %5.2f\n", steps);
    //         }
    //     }
    // }

    // if (isSet(shift, toggled, BTN_STEP_DEC)) {
    //     if (_pGait) {
    //         float steps = _pGait->getStepsPerSec() - 0.5f;
    //         if (steps > 0) {
    //             _pGait->setStepsPerSec(steps);
    //             LOG("steps per sec : %5.2f\n", steps);
    //         }
    //     }
    // }

    // if (isSet(shift, toggled, BTN_STEP_XY_DEC)) {
    //     if (_pGait->getStep()->x > 0) {
    //         _pGait->getStep()->x--;
    //     }
    //     if (_pGait->getStep()->y > 0) {
    //         _pGait->getStep()->y--;
    //     }
    //     if (_pGait) {
    //         _pGait->getStep()->dump("STEP");
    //     }
    // }

    // if (isSet(shift, toggled, BTN_STEP_XY_INC)) {
    //     if (_pGait->getStep()->x < 50) {
    //         _pGait->getStep()->x++;
    //     }
    //     if (_pGait->getStep()->y < 50) {
    //         _pGait->getStep()->y++;
    //     }
    //     if (_pGait) {
    //         _pGait->getStep()->dump("STEP");
    //     }
    // }

    if (isSet(shift, toggled, BTN_BALANCE)) {
        _isBalance = (switches & BTN_BALANCE);
        LOG("balance mode:%d\n", int(_isBalance));
#if CONFIG_ENABLE_GYRO
        getGyroProc()->reset();
#endif
    }

    if (isSet(shift, toggled, BTN_SAVE)) {
        saveUserData();
    }

    if (isSet(shift, toggled, BTN_LOAD)) {
        loadUserData();
    }
}

void CommandHandler::setButtons(int btns, bool toggle) {
    static int switches = 0;
    static int lastBtns = 0;

    int  shift     = btns & (BTN_SHIFT_L | BTN_SHIFT_R);
    int  others    = btns & ~(BTN_SHIFT_L | BTN_SHIFT_R);
    int  toggledOn = (others ^ lastBtns) & others;

    if (shift || toggledOn) {
        switches = switches ^ toggledOn;
        setSwitches(shift | switches);
    }
    lastBtns = toggle ? 0 : others;
}

void CommandHandler::onRC(struct param_rc* rc) {
    static unsigned long lastProtoCall;
    unsigned long ts = millis();

    if (rc->flag == CONTROLLER_PROTOCOL) {
        if (!isConnected(CONTROLLER_PROTOCOL)) {
            setConnected(CONTROLLER_PROTOCOL, true);
        }
        lastProtoCall = ts;

        // btpad overrides control
        if (isConnected(CONTROLLER_BTPAD))
            return;

        setSwitches(aux2Switches(rc));
    }

    if (IS_ELAPSED(ts, lastProtoCall, 1000)) {
        if (isConnected(CONTROLLER_PROTOCOL)) {
            setConnected(CONTROLLER_PROTOCOL, false);
        }
    }

    _rc = *rc;
    if (rc->flag == CONTROLLER_BTPAD) {
        // dead zone
        _rc.pitch    = filterDeadZone(_rc.pitch);
        _rc.roll     = filterDeadZone(_rc.roll);
        _rc.throttle = filterDeadZone(_rc.throttle);
        _rc.yaw      = filterDeadZone(_rc.yaw);
    }
    //LOG("RC:%5d %5d %5d %5d\n", _rc.yaw, _rc.throttle, _rc.roll, _rc.pitch);
}

void CommandHandler::onAttitude(struct param_att* att) {
}


uint8_t CommandHandler::onBattDV(void) {
    return (uint8_t)0;
}

int8_t CommandHandler::onOthers(uint8_t cmd, uint8_t* pData, uint8_t size, uint8_t* pRes) {
    int8_t  ret = -1;
    /*
            if (cmd == 75) {
                int leg = *pData++;

                s16 *pAngles = (s16*)pData;
                s16 a10  = (s16)(*pAngles++);
                s16 b10  = (s16)(*pAngles++);
                s16 c10  = (s16)(*pAngles++);
            }
    */
    return ret;
}

void CommandHandler::saveUserData() {
    // if (!_pGait)
    //     return;

    // File file = SPIFFS.open(FILE_STEP_CFG, FILE_WRITE);
    // if (!file) {
    //     LOG("file write open error\n");
    //     return;
    // }
    // file.write((uint8_t*)_pGait->getStep(), sizeof(Vector));

    // // steps per sec
    // float val = _pGait->getStepsPerSec();
    // file.write((uint8_t*)&val, sizeof(float));
    // file.close();
    // LOG("user data SAVED !!!\n");
}

void CommandHandler::loadUserData() {
    // if (!_pGait)
    //     return;

    // File file = SPIFFS.open(FILE_STEP_CFG);
    // if (!file) {
    //     LOG("file not found\n");
    //     return;
    // }

    // if (file.read((uint8_t*)_pGait->getStep(), sizeof(Vector)) > 0) {
    //     _pGait->getStep()->dump("STEP");
    // }

    // float val;
    // if (file.read((uint8_t*)&val, sizeof(float)) > 0) {
    //     val = round(val);
    //     _pGait->setStepsPerSec(val);
    //     LOG("setStepsPerSec:%5.2f\n", val);
    // }

    // file.close();
    // LOG("user data LOADED !!!\n");
}

uint16_t CommandHandler::filterDeadZone(uint16_t val) {
    if (1480 <= val && val <= 1520) {
        val = 1500;
    }
    return val;
}

uint16_t CommandHandler::aux2CombBits(uint16_t val, uint8_t bits) {
    uint16_t div = BV(bits) - 1;

    val -= 1000;
    return val * div / 1000;
}

uint16_t CommandHandler::aux2Switches(struct param_rc* rc) {
    uint16_t btn  = 0;
    uint16_t comb = 0;

    for (uint8_t i = 0; i < 8; i++) {
        switch (i) {
            case 0:
                if (rc->aux[i] > 1500) {
                    btn |= _BV(ControlStick::BTN_L1);
                }
                break;

            case 1:
                if (rc->aux[i] > 1500) {
                    btn |= _BV(ControlStick::BTN_L2);
                }
                break;

            case 2:
                comb = aux2CombBits(rc->aux[i], 4);
                if (comb & 0x04) {
                    btn |= _BV(ControlStick::BTN_START);
                }
                if (comb & 0x08) {
                    //btn |= _BV(ControlStick::BTN_LTHUMB);
                }
                if (comb & 0x01) {
                    btn |= _BV(ControlStick::BTN_Y);
                }
                if (comb & 0x02) {
                    btn |= _BV(ControlStick::BTN_X);
                }
                break;

            case 3:
                if (rc->aux[i] > 1500) {
                    btn |= _BV(ControlStick::BTN_R1);
                }
                break;

            case 4:
                if (rc->aux[i] > 1500) {
                    btn |= _BV(ControlStick::BTN_R2);
                }
                break;

            case 5:
                comb = aux2CombBits(rc->aux[i], 4);
                if (comb & 0x04) {

                }
                if (comb & 0x08) {
                    //btn |= _BV(ControlStick::BTN_RTHUMB);
                }
                if (comb & 0x01) {
                    btn |= _BV(ControlStick::BTN_B);
                }
                if (comb & 0x02) {
                    btn |= _BV(ControlStick::BTN_A);
                }
                break;

            case 6:
                if (rc->aux[i] < 1200) {
                    btn |= _BV(ControlStick::BTN_DPAD_LEFT);
                } else if (rc->aux[i] > 1800) {
                    btn |= _BV(ControlStick::BTN_DPAD_RIGHT);
                }
                break;

            case 7:
                if (rc->aux[i] < 1200) {
                    btn |= _BV(ControlStick::BTN_DPAD_DOWN);
                } else if (rc->aux[i] > 1800) {
                    btn |= _BV(ControlStick::BTN_DPAD_UP);
                }
                break;
        }
    }
    return btn;
}

void CommandHandler::updateStatusLeds() {
    static int iOldState = -1;
    static int iOldBlink = -1;

    const StatusLed::color_idx_t colorStatus[] = {
        StatusLed::COLOR_BLUE,          // bt connecting
        StatusLed::COLOR_LIGHTBLUE,     // no walk
        StatusLed::COLOR_PURPLE,        // trot
        StatusLed::COLOR_GREEN,         // wave
        StatusLed::COLOR_CYAN,          // ripple
    };

    int  state;
    int  blink = 0;
    bool isConnected = _isConnected[0] | _isConnected[1];
    StatusLed::color_idx_t  offBlink = StatusLed::COLOR_BLACK;

    if (isConnected) {
        if (!_isWalk) {
            state = 1;
        } else {
            state = 2 + _nGait;
        }
        if (_isBalance) {
            blink = 500;
            offBlink = StatusLed::COLOR_PINK;
        } else if (_isFlash) {
            blink = 200;
            offBlink = StatusLed::COLOR_BLACK;
        }
    } else {
        state = 0;
        blink = 500;
    }

    if (state != iOldState || blink != iOldBlink) {
        // _statusLed.set(StatusLed::ALL_POS, colorStatus[state], blink, offBlink);
        iOldState = state;
        iOldBlink = blink;
    }
}
