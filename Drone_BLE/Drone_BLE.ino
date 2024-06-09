#include "config.h"

BLEService LEDService("19b10000-e8f2-537e-4f6c-d104768a1214"); // Service UUID
BLEIntCharacteristic joyStick1Characteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic joyStick2Characteristic("19b10003-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic startButtonCharacteristic("19b10009-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

int desiredThrottle = STABLE;
int desiredYaw = 0;
int desiredPitch = 0;
int desiredRoll = 0;

int state = STOP;

int motorSpeed[4];

void handler1(BLEDevice central, BLECharacteristic characteristic){
//   signed char values[2];
//   characteristic.readValue(values, 2);

//   Serial.write("JoyStick 1 X: ");
//   Serial.print(values[0], DEC);
//   Serial.write("JoyStick 1 Y: ");
//   Serial.print(values[1], DEC);
//   Serial.write("\n");
}

int normalize(int x) {
  // return ((x+100.0)/200.0) * 128;
  return IDDLE + (((x))/100.0) * 50;
}

void leding_time(int val1, int val2) {
  // Serial.print(val1, DEC);
  // Serial.write(" ");
  // Serial.print(val2, DEC);
  // Serial.write("\n");
  
  // if( (val1) == 0 ) {
  //   allEngines(IDDLE_OFF);
  //   return;
  // }
  // if( (val1) < 0 ) {
  //   myLedWrite(channelLeftUp, IDDLE); 
  //   return;
  // }
  // myLedWrite(channelLeftUp, normalize(val1)); 
  
  // allEngines(normalize(val1));
  desiredThrottle = normalize(val1);
}

void handler2(BLEDevice central, BLECharacteristic characteristic){
  signed char values[2];
  characteristic.readValue(values, 2);
  leding_time(values[0], values[1]);
  
  // Serial.write("\n");

  // Serial.write("JoyStick 2 X: ");
  // Serial.print(values[0], DEC);
  // Serial.write("JoyStick 2 Y: ");
  // Serial.print(values[1], DEC);

  // Serial.write("\n");
}

void startButtonHandler(BLEDevice central, BLECharacteristic characteristic) {
  signed char value[1];
  characteristic.readValue(value, 1);
  
  Serial.print(*value, DEC);
  switch(*value) {
    case 0:
      state = STOP;
      desiredThrottle = IDDLE;
      allEngines(0);
    break;
    case 1:
      Serial.write("IT'S MORBING TIME \n");
      state = FLY;
      startUpEngines();
    break;
  }

}

void stablize() {
  if( state != FLY )
    return;
  // float yawError = desiredYaw - mpu.getAngleX();
  float pitchError = desiredPitch +  mpu.getAngleY();
  float rollError = desiredRoll + mpu.getAngleX();
  float desiredAltitude = desiredThrottle - 0;
  
  float kp = 0.5; // Proportional gain, tune this value
  motorSpeed[0] = kp * (-pitchError + rollError + desiredAltitude);
  motorSpeed[1] = kp * (-pitchError - rollError + desiredAltitude);
  motorSpeed[2] = kp * (pitchError - rollError + desiredAltitude);
  motorSpeed[3] = kp * (pitchError + rollError + desiredAltitude);

  myLedWrite(channelLeftUp, motorSpeed[0]); 
  myLedWrite(channelRightUp, motorSpeed[1]); 
  myLedWrite(channelRigthDown, motorSpeed[2]); 
  myLedWrite(channelLeftDown, motorSpeed[3]); 

	// Serial.println("\n");
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
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero


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
  Serial.write("Start\n"); 


  // allEngines(IDDLE_OFF);
  // state = FLY;
  //startUpEngines();


  
  
                                                // start advertising
}
 
void loop() {
  BLEDevice central = BLE.central();                            // listen for BLE devices to connect:                                                         //end of while loop
                                                                // you can put code here for what todo when not connected

  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
	// Serial.print("X : ");
	// Serial.print(mpu.getAngleX());
	// Serial.print("\tY : ");
	// Serial.print(mpu.getAngleY());
	// Serial.print("\tZ : ");
	// Serial.println(mpu.getAngleZ());
	timer = millis();  
  stablize();
  }

  //  allEngines(IDDLE_OFF);
}                   