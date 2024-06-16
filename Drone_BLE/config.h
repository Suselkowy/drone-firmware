#ifndef MY_CFG
#define MY_CFG

#include <ArduinoBLE.h>
#include "esp32-hal.h"
#include "engines.h"

// #define DEBUG
// #define INTEGRAL

static const int pinLeftUp = D7;
static const int channelLeftUp = 4;

static const int pinRightUp = D8;
static const int channelRightUp = 2;

static const int pinRigthDown = D9;
static const int channelRigthDown = 3;

static const int pinLeftDown = D10;
static const int channelLeftDown = 1;

#define IDDLE 128 // 120
#define STABLE IDDLE + 37 // 120
#define IDDLE_OFF 120
#define START_PWM 240
#define BIAS_TYL 9
#define PWM_HZ 400
#define PWM_RES 8

#endif