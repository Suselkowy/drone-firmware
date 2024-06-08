#ifndef MY_CFG
#define MY_CFG

#include <ArduinoBLE.h>
#include "esp32-hal.h"
#include "engines.h"

static const int pinLeftUp = D7;
static const int channelLeftUp = 4;

static const int pinLeftDown = D8;
static const int channelLeftDown = 1;

static const int pinRightUp = D9;
static const int channelRightUp = 2;

static const int pinRigthDown = D10;
static const int channelRigthDown = 3;

#define IDDLE 128 // 120
#define IDDLE_OFF 120
#define START_PWM 240
#define PWM_HZ 400
#define PWM_RES 8

#endif