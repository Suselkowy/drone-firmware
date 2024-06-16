#include "config.h"

BLEService LEDService("19b10000-e8f2-537e-4f6c-d104768a1214"); // Service UUID
BLEIntCharacteristic joyStick1Characteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic joyStick2Characteristic("19b10003-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic startButtonCharacteristic("19b10009-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic pulseCharacteristic("19b10010-e8f2-537e-4f6c-d104768a1214", BLERead | BLEWrite | BLENotify);

#include "Wire.h"
#include <MPU6050_light.h>
// #include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

MPU6050 mpu(Wire);
// Adafruit_BMP280 bmp; 
unsigned long timer = 0;
unsigned long pulseTimer = 0;

int desiredThrottle = 0;
int desiredYaw = 0;
int desiredPitch = 0;
int desiredRoll = 0;

float yawErrorSum = 0;
float pitchErrorSum = 0;
float rollErrorSum = 0;

int state = STOP;

int motorSpeed[4];

void clearIntegral() {
  yawErrorSum = 0;
  pitchErrorSum = 0;
  rollErrorSum = 0;
}

void stop() {
  state = STOP;
  desiredThrottle = 0;
  desiredYaw = 0;
  desiredPitch = 0;
  desiredRoll = 0;
  clearIntegral();

  allEngines(0);
}

int normalize(int x) {
  // return ((x+100.0)/200.0) * 128;
  return (((x))/100.0) * 30;
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
      if(state != ON ) {
        break;
      }
      Serial.write("IT'S MORBING TIME \n");
      desiredThrottle = 0;
      clearIntegral();
      state = FLY;
    break;
    case 2:
      Serial.println("-1");
      clearIntegral();
      mpu.angleX = 0;
      mpu.angleY = 0;
      mpu.angleZ = 0;
      mpu.calcOffsets();
    break;
    case 3:
      Serial.write("ON \n");
      startUpEngines();
      desiredThrottle = 0;
      state = ON;
    break;
  }

}

void stablize() {
  if( state != FLY )
    return;
  float yawError = desiredYaw - mpu.getAngleZ();
  yawError =  min(max(yawError, -60.0f), 60.0f);
  // yawError = 0;
  
  float pitchError = desiredPitch + mpu.getAngleY();
  float rollError = desiredRoll + mpu.getAngleX();
  float desiredAltitude = 0 - 0;

  // yawError = 0;
  // pitchError = 0;
  // rollError = 0;
  
  float kp = 1.0; // Proportional gain, tune this value
  motorSpeed[0] = STABLE + kp * (pitchError - rollError + desiredAltitude + yawError) + desiredThrottle + BIAS_TYL;
  motorSpeed[1] = STABLE + kp * (pitchError + rollError + desiredAltitude - yawError) + desiredThrottle + BIAS_TYL + 5;
  motorSpeed[2] = STABLE + kp * (-pitchError + rollError + desiredAltitude + yawError) + desiredThrottle - BIAS_TYL;
  motorSpeed[3] = STABLE + kp * (-pitchError - rollError + desiredAltitude - yawError) + desiredThrottle - BIAS_TYL;

  #ifdef INTEGRAL

  #define MAX_INTEGRAL 100.0f
  #define PRZELICZNIK_KUMULACJI 0.1f

  pitchErrorSum += PRZELICZNIK_KUMULACJI * pitchError;
  rollErrorSum += PRZELICZNIK_KUMULACJI * rollError;
  // yawErrorSum += PRZELICZNIK_KUMULACJI * yawError;

  pitchErrorSum = min(max(pitchErrorSum, -MAX_INTEGRAL), MAX_INTEGRAL);
  rollErrorSum = min(max(rollErrorSum, -MAX_INTEGRAL), MAX_INTEGRAL);
  yawErrorSum =  min(max(yawErrorSum, -MAX_INTEGRAL), MAX_INTEGRAL);

  #define MAX_INTEGRAL_MOTOR 24.0f

  float ki = 0.2; // Proportional gain, tune this value
  motorSpeed[0] += min( ki * (pitchErrorSum - rollErrorSum + yawErrorSum), MAX_INTEGRAL_MOTOR );
  motorSpeed[1] += min( ki * (pitchErrorSum + rollErrorSum - yawErrorSum), MAX_INTEGRAL_MOTOR );
  motorSpeed[2] += min( ki * (-pitchErrorSum + rollErrorSum + yawErrorSum), MAX_INTEGRAL_MOTOR );
  motorSpeed[3] += min( ki * (-pitchErrorSum - rollErrorSum - yawErrorSum), MAX_INTEGRAL_MOTOR );

  #endif

  for( int i = 0 ; i < sizeof(motorSpeed)/sizeof(int); ++i ) {
    motorSpeed[i] = min(max(motorSpeed[i], IDDLE_OFF),220);
  }

  myLedWrite(channelLeftUp, motorSpeed[0]); 
  myLedWrite(channelRightUp, motorSpeed[1]); 
  myLedWrite(channelRigthDown, motorSpeed[2]); 
  myLedWrite(channelLeftDown, motorSpeed[3]); 

  #ifdef DEBUG
	Serial.println("\n");
  #endif
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

  // unsigned bmpStatus;
  // //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // bmpStatus = bmp.begin(0x76);
  // if (!bmpStatus) {
  //   Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
  //                     "try a different address!"));
  //   Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
  //   Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //   Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //   Serial.print("        ID of 0x60 represents a BME 280.\n");
  //   Serial.print("        ID of 0x61 represents a BME 680.\n");
  //   while (1) delay(10);
  // }

  // /* Default settings from datasheet. */
  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //                 Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //                 Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  //                 Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  //                 Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  // mpu.setAccOffsets(-0.15, -0.07, -1.33);
  // mpu.setGyroOffsets(0.57 , 0.68, -0.34);

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
 
  if((millis()-timer)>6){ // print data every 10ms
  
#ifdef DEBUG  

    // Serial.print("AX : ");
    // Serial.print(mpu.getAccXoffset());
    // Serial.print("\tAY : ");
    // Serial.print(mpu.getAccYoffset());
    // Serial.print("\tAZ : ");
    // Serial.println(mpu.getAccZoffset());
    // Serial.print("GX : ");
    // Serial.print(mpu.getGyroXoffset());
    // Serial.print("\tGY : ");
    // Serial.print(mpu.getGyroYoffset());
    // Serial.print("\tGZ : ");
    // Serial.println(mpu.getGyroZoffset());


    // Serial.print("X : ");
    // Serial.print(mpu.getAngleX());
    // Serial.print("\tY : ");
    // Serial.print(mpu.getAngleY());
    // Serial.print("\tZ : ");
    // Serial.println(mpu.getAngleZ());
    // Serial.println(bmp.readAltitude(1013.25));

    Serial.print("X : ");
    Serial.print(rollErrorSum);
    Serial.print("\tY : ");
    Serial.print(pitchErrorSum);
    Serial.print("\tZ : ");
    Serial.println(yawErrorSum);
#endif
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