#include "engines.h"

void myLedWrite(int channel, int duty) {
  Serial.print(duty, DEC);
  Serial.write("\n");
  ledcWrite(channel,duty);
}

void setUpPin(int pin, int channel) {
  ledcSetup(channel, PWM_HZ, PWM_RES);
  ledcAttachPin(pin, channel);  
  myLedWrite(channel, IDDLE_OFF);
}

void setUpPwm() {
  setUpPin(pinLeftUp, channelLeftUp);
  setUpPin(pinLeftDown, channelLeftDown);
  setUpPin(pinRightUp, channelRightUp);
  setUpPin(pinRigthDown, channelRigthDown);
}

void allEngines(int duty) {
    myLedWrite(channelLeftUp, duty); 
    myLedWrite(channelLeftDown, duty); 
    myLedWrite(channelRightUp, duty); 
    myLedWrite(channelRigthDown, duty); 
}


void startEngine(int channel) {
  myLedWrite(channel, IDDLE_OFF); 
  delay(500);
  myLedWrite(channel, START_PWM); 
  delay(500);
  myLedWrite(channel, IDDLE_OFF); 
}

void startUpEngines() {
  allEngines(IDDLE_OFF); 
  delay(500);
  allEngines(START_PWM); 
  delay(500);
  allEngines(IDDLE_OFF); 
}