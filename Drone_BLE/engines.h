#ifndef MY_ENGINES
#define MY_ENGINES
#include "config.h"
#include "engines.h"

enum { STOP, FLY };

void myLedWrite(int channel, int duty);
void setUpPin(int pin, int channel);
void setUpPwm();
void allEngines(int duty);
void startEngine(int channel);
void startUpEngines();

#endif