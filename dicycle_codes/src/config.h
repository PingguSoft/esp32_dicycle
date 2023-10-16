#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>

/*
*****************************************************************************************
* FEATURES
*****************************************************************************************
*/
#define CONFIG_CONTROL_BTPADS   1
#define CONFIG_CONTROL_BT_UART  2
#define CONFIG_CONTROL          CONFIG_CONTROL_BTPADS

#define CONFIG_ENABLE_GYRO      1

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
#define UNIT_MM             1.0f    // (unit:mm)
#define PRECISION           0.1f    // (unit:mm)
#define MOTION_SMOOTH_GAIN  0.95f

#define BODY_NUM_LEGS       2
#define BODY_NUM_JOINTS     2

#define BODY_UPPER_LEG_LEN  95
#define BODY_LOWER_LEG_LEN  75

#define SERVER_NAME             "dicycle"
#define FILE_SERVO_CFG          "/offs.cfg"
#define FILE_STEP_CFG           "/step.cfg"
#define FILE_GYRO_CFG           "/gyro.cfg"

#define BODY_MAX_YAW            30.0f   // (unit:degree)
#define BODY_MAX_PITCH          30.0f
#define BODY_MAX_ROLL           30.0f

#define BODY_MAX_PITCH_OUT      500
#define BODY_MAX_ROLL_OUT       100
#define BODY_MAX_MOVE_MULT      3.0f

#define WIFI_SSID               "TJ's Library"
#define WIFI_PASSWORD           "cafebabe12"


#define WHEEL_RADIUS_MM         36
#define AXLE_WIDTH_MM           180
#define AXLE_HALF_WIDTH_MM      (AXLE_WIDTH_MM / 2)
#define TICKS_PER_CYCLE         360

/*
*****************************************************************************************
* H/W CONSTANTS (PINS)
*****************************************************************************************
*/
#define PIN_SCL0                22
#define PIN_SDA0                21

#define PIN_L_PHASE_A           15
#define PIN_L_PHASE_B           2
#define PIN_L_PWM               4

#define PIN_R_PHASE_A           16
#define PIN_R_PHASE_B           17
#define PIN_R_PWM               5

#define PIN_LED_STRIP           12
#define PIN_PWR_LED             0
#define PIN_PWR_ADC             A0

// UART2
#define PIN_RXD2                13
#define PIN_TXD2                14      // 12 should be LOW during boot so TXD2 is changed to 14

// H/W CONFIGURATION
#define HW_SERVO_UPDATE_FREQ    50

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/


#define LOG(...)        printf(__VA_ARGS__)
#define ARRAY_SIZE(x)   (sizeof(x) / sizeof((x)[0]))

#define IS_FRONT_LEG(leg)       (leg == 0 || leg == 5)
#define IS_RIGHT_LEG(leg)       (leg < 3)


#endif
