#include <ArduinoBLE.h>
#include "esp32-hal.h"

int pinLeftUp = D7;
int channelLeftUp = 4;

int pinLeftDown = D8;
int channelLeftDown = 1;

int pinRightUp = D9;
int channelRightUp = 2;

int pinRigthDown = D10;
int channelRigthDown = 3;

#define IDDLE 128 // 120
#define IDDLE_OFF 120
#define START_PWM 240
#define PWM_HZ 400
#define PWM_RES 8

typedef int unit8_t;
 
BLEService LEDService("19b10000-e8f2-537e-4f6c-d104768a1214"); // Service UUID
BLEIntCharacteristic joyStick1Characteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic joyStick2Characteristic("19b10003-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic startButtonCharacteristic("19b10009-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);

void myLedWrite(int channel, int duty) {
  Serial.print(duty, DEC);
  Serial.write("\n");
  ledcWrite(channel,duty);
}

void setUpPin(unit8_t pin, uint8_t channel) {
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

void startEngine(uint8_t channel) {
  myLedWrite(channel, IDDLE_OFF); 
  delay(500);
  myLedWrite(channel, START_PWM); 
  delay(500);
  myLedWrite(channel, IDDLE_OFF); 
}

void startUpEngines() {
  // startEngine(channelLeftUp);
  // startEngine(channelLeftDown);
  // startEngine(channelRightUp);
  // startEngine(channelRigthDown);
  allEngines(IDDLE_OFF); 
  delay(500);
  allEngines(START_PWM); 
  delay(500);
  allEngines(IDDLE_OFF); 
}

void allEngines(int duty) {
    myLedWrite(channelLeftUp, duty); 
    myLedWrite(channelLeftDown, duty); 
    myLedWrite(channelRightUp, duty); 
    myLedWrite(channelRigthDown, duty); 
}


void handler1(BLEDevice central, BLECharacteristic characteristic){
  signed char values[2];
  characteristic.readValue(values, 2);




  Serial.write("JoyStick 1 X: ");
  Serial.print(values[0], DEC);
  Serial.write("JoyStick 1 Y: ");
  Serial.print(values[1], DEC);
  Serial.write("\n");
}

int normalize(int x) {
  // return ((x+100.0)/200.0) * 128;
  return IDDLE + (((x))/100.0) * 100;
}

void leding_time(int val1, int val2) {
  Serial.print(val1, DEC);
  Serial.write(" ");
  Serial.print(val2, DEC);
  Serial.write("\n");
  
  if( (val1) == 0 ) {
    allEngines(IDDLE_OFF);
    return;
  }
  // if( (val1) < 0 ) {
  //   myLedWrite(channelLeftUp, IDDLE); 
  //   return;
  // }
  myLedWrite(channelLeftUp, normalize(val1)); 
  
  allEngines(normalize(val1));
}

void handler2(BLEDevice central, BLECharacteristic characteristic){
  signed char values[2];
  characteristic.readValue(values, 2);
  leding_time(values[0], values[1]);
  
  Serial.write("\n");

  Serial.write("JoyStick 2 X: ");
  Serial.print(values[0], DEC);
  Serial.write("JoyStick 2 Y: ");
  Serial.print(values[1], DEC);

  Serial.write("\n");
}

void startButtonHandler(BLEDevice central, BLECharacteristic characteristic) {
  signed char values[1];
  characteristic.readValue(values, 1);
  Serial.write("IT'S MORBING TIME \n");
  // startEngine(channelLeftUp);
  startUpEngines();
}


void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void setup() {





  Serial.begin(9600);
  if (!BLE.begin()) {                                           // begin initialization
    while (1);                                                  // wait until initialization complete
  }

  setUpPwm();

  BLE.setLocalName("ESP123");                                  // set advertised local name
  BLE.setAdvertisedService(LEDService);                    // set advertised service UUID
  LEDService.addCharacteristic(joyStick1Characteristic);
  joyStick1Characteristic.setEventHandler(BLEWritten, handler1);     
  LEDService.addCharacteristic(joyStick2Characteristic);
  joyStick2Characteristic.setEventHandler(BLEWritten, handler2);  
  LEDService.addCharacteristic(startButtonCharacteristic);
  startButtonCharacteristic.setEventHandler(BLEWritten, startButtonHandler);  
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler); 
  BLE.addService(LEDService);                                   // add service
  BLE.advertise();
  Serial.write("Start\n");                                                 // start advertising
}
 
void loop() {
  BLEDevice central = BLE.central();                            // listen for BLE devices to connect:                                                         //end of while loop
                                                                // you can put code here for what todo when not connected
}                   