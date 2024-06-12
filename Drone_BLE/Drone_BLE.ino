#include "config.h"

BLEService LEDService("19b10000-e8f2-537e-4f6c-d104768a1214"); // Service UUID
BLEIntCharacteristic joyStick1Characteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic joyStick2Characteristic("19b10003-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic startButtonCharacteristic("19b10009-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic pulseCharacteristic("19b10010-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);

#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

MPU6050 mpu(Wire);
Adafruit_BMP280 bmp; 
unsigned long timer = 0;
unsigned long pulseTimer = 0;

int desiredThrottle = 0;
int desiredYaw = 0;
int desiredPitch = 0;
int desiredRoll = 0;

int state = STOP;

int motorSpeed[4];

void stop() {
  state = STOP;
  desiredThrottle = 0;
  desiredYaw = 0;
  desiredPitch = 0;
  desiredRoll = 0;
  allEngines(0);
}

int normalize(int x) {
  // return ((x+100.0)/200.0) * 128;
  return (((x))/100.0) * 40;
}

void leding_time(int val1, int val2) {
  // Serial.print(val1, DEC);
  // Serial.write(" ");
  // Serial.print(val2, DEC);
  // // Serial.write("\n");
  // desiredThrottle = normalize(val2);
}

void handler1(BLEDevice central, BLECharacteristic characteristic){
  signed char values[2];
  characteristic.readValue(values, 2);
  // leding_time(values[0], values[1]);
  desiredThrottle = normalize(-values[1]);
  
  // Serial.write("\n");

  // Serial.write("JoyStick 2 X: ");
  // Serial.print(values[0], DEC);
  // Serial.write("JoyStick 2 Y: ");
  // Serial.print(values[1], DEC);

  // Serial.write("\n");
}

void handler2(BLEDevice central, BLECharacteristic characteristic){
  signed char values[2];
  characteristic.readValue(values, 2);
  desiredRoll = normalize(-values[0])/3.0;
  desiredPitch = normalize(values[1])/3.0;
//   Serial.write("JoyStick 1 X: ");
//   Serial.print(values[0], DEC);
//   Serial.write("JoyStick 1 Y: ");
//   Serial.print(values[1], DEC);
//   Serial.write("\n");
}

void pulseHandler(BLEDevice central, BLECharacteristic characteristic){
  signed char values[1];
  characteristic.readValue(values, 1);
  Serial.println("p");
  pulseTimer = millis();
}


void startButtonHandler(BLEDevice central, BLECharacteristic characteristic) {
  signed char value[1];
  characteristic.readValue(value, 1);
  
  Serial.print(*value, DEC);
  switch(*value) {
    case 0:
      stop();
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
  float pitchError = desiredPitch - mpu.getAngleY();
  float rollError = desiredRoll - mpu.getAngleX();
  float desiredAltitude = desiredThrottle - 0;
  
  float kp = 0.35; // Proportional gain, tune this value
  motorSpeed[0] = STABLE + kp * (pitchError - rollError + desiredAltitude);
  motorSpeed[1] = STABLE + kp * (pitchError + rollError + desiredAltitude);
  motorSpeed[2] = STABLE + kp * (-pitchError + rollError + desiredAltitude);
  motorSpeed[3] = STABLE + kp * (-pitchError - rollError + desiredAltitude);

  for( int i = 0 ; i < sizeof(motorSpeed)/sizeof(int); ++i ) {
    motorSpeed[i] = min(max(motorSpeed[i], IDDLE),255);
  }

  myLedWrite(channelLeftUp, motorSpeed[0]); 
  myLedWrite(channelRightUp, motorSpeed[1]); 
  myLedWrite(channelRigthDown, motorSpeed[2]); 
  myLedWrite(channelLeftDown, motorSpeed[3]); 

	Serial.println("\n");
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  stop();
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  unsigned bmpStatus;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmpStatus = bmp.begin(0x76);
  if (!bmpStatus) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero

  setUpPwm();

  if (!BLE.begin()) {                                           // begin initialization
    while (1);                                                  // wait until initialization complete
  }

  BLE.setLocalName("ESP123");                                  // set advertised local name
  BLE.setAdvertisedService(LEDService);                    // set advertised service UUID
  LEDService.addCharacteristic(joyStick1Characteristic);
  joyStick1Characteristic.setEventHandler(BLEWritten, handler1);     
  LEDService.addCharacteristic(joyStick2Characteristic);
  joyStick2Characteristic.setEventHandler(BLEWritten, handler2);  
  LEDService.addCharacteristic(startButtonCharacteristic);
  startButtonCharacteristic.setEventHandler(BLEWritten, startButtonHandler);  
  LEDService.addCharacteristic(pulseCharacteristic);
  pulseCharacteristic.setEventHandler(BLEWritten, pulseHandler);  

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler); 
  BLE.addService(LEDService);                                   // add service
  BLE.advertise();
  Serial.write("Start\n"); 


  // allEngines(IDDLE_OFF);
  // state = FLY;
  // startUpEngines();
}
 
void loop() {
  BLE.central();

  mpu.update();  
  if((millis()-timer)>100){ // print data every 10ms
    // Serial.print("X : ");
    // Serial.print(mpu.getAngleX());
    // Serial.print("\tY : ");
    // Serial.print(mpu.getAngleY());
    // Serial.print("\tZ : ");
    // Serial.println(mpu.getAngleZ());
    Serial.println(bmp.readAltitude(1013.25));
    timer = millis();  
    if( state == FLY ) {
      stablize();
    }
  }


  // if((millis()-pulseTimer)>3000){ 
  //   stop();
  //   pulseTimer = millis();
  // }
}                   