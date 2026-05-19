#include <Arduino.h>
#include <Wire.h>

#include "Vmath.h"
#include "Qmath.h"
#include "FrameBuffers.h"
#include "ArduinoDevices.h"

/*
This script reads the value of a sensor from an IMU
*/

// Innitialize IMU in the global scope or it will be dropped!
IMU_ICM20649 imu;

void setup() {
  Serial.begin(115200);
  // This will wait until the arduino is plugged in to serial to begin
  while(!Serial) delay(10);
  Serial.println("Testing Serial Communication");
  // While you can just call imu.init() by itself, if you do it like this you can customize error handling
  // To change the IMU settings ctrl+click the init method to go to source
  if(!imu.init()){
    Serial.println("failed to innitialize sensor");
    while(1) delay(10);
  }
}

void loop() {
  // IMU values will NOT UPDATE UNTIL YOU CALL THIS FUNCTION
  // It returns you the timestamp of sampling that can be used for deterministic timing
  unsigned long updateTime = imu.updateimu();
  // This is the minimal setup to create a new bframe, just call its constructor
  Bframe rawData = Bframe(updateTime);
  // The imu can take a bframe reference and fill out all the details
  imu.fillBframeWithRawData(rawData);

  Serial.print("Sample Time : ");
  Serial.print(rawData.timestamp / 1e6);
  Serial.print(" s ");
  Serial.print(" Acceleration in X axis :  ");
  Serial.println(rawData.acceleration.x);
  delay(100);
}