#include <ArduinoBLE.h>
#include "esp32-hal.h"

int led1 = D7;
int channel = 0;

#define IDDLE 128
#define IDDLE_OFF 120
 
BLEService LEDService("19b10000-e8f2-537e-4f6c-d104768a1214"); // Service UUID
BLEIntCharacteristic joyStick1Characteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic joyStick2Characteristic("19b10003-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);

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
  return IDDLE + ((abs(x))/100.0) * 100;
}

void leding_time(int val1, int val2) {
  Serial.print(val1, DEC);
  Serial.write(" ");
  Serial.print(val2, DEC);
  Serial.write("\n");
  
  if( (val1) == 0 ) {
    ledcWrite(channel, IDDLE_OFF); 
    return;
  }
  if( (val1) < 0 ) {
    ledcWrite(channel, IDDLE); 
    return;
  }
  ledcWrite(channel, normalize(val1)); 
}

void handler2(BLEDevice central, BLECharacteristic characteristic){
  signed char values[2];
  characteristic.readValue(values, 2);
  leding_time(values[0], values[1]);
  
  Serial.print(normalize(values[0]), DEC);
  Serial.write("\n");

  Serial.write("JoyStick 2 X: ");
  Serial.print(values[0], DEC);
  Serial.write("JoyStick 2 Y: ");
  Serial.print(values[1], DEC);

  Serial.write("\n");
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
  ledcSetup(channel, 400, 8);
  ledcAttachPin(led1, channel);  
  ledcWrite(channel, IDDLE_OFF);
  BLE.setLocalName("ESP123");                                  // set advertised local name
  BLE.setAdvertisedService(LEDService);                    // set advertised service UUID
  LEDService.addCharacteristic(joyStick1Characteristic);
  joyStick1Characteristic.setEventHandler(BLEWritten, handler1);     
  LEDService.addCharacteristic(joyStick2Characteristic);
  joyStick2Characteristic.setEventHandler(BLEWritten, handler2);  
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler); 
  BLE.addService(LEDService);                                   // add service
  BLE.advertise();
  Serial.write("Start");                                                 // start advertising
}
 
void loop() {
  BLEDevice central = BLE.central();                            // listen for BLE devices to connect:                                                         //end of while loop
                                                                // you can put code here for what todo when not connected
}                                                               // end of loop
