#include <Arduino.h>
#include <Wire.h>

#include "Vmath.h"
#include "Qmath.h"
#include "FrameBuffers.h"
#include "ArduinoDevices.h"

/*
This script is an example of how to use the time history of sensor to take a derivative
*/

// Innitialize IMU in the global scope or it will be dropped!
IMU_ICM20649 imu;

// Innitialize the first Bframe pointer here
Bframe* head = new Bframe(0, 1);
Quat vehicle_orientation = Quat(1.0f,0.0f,0.0f,0.0f);

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
  // This time when we make a Bframe we create a bframe pointer, this is so the Bframe is not dropped from the call stack when loop() is rerun
  // Furthermore we define a maximum frame depth or history explicitly.
  Bframe* rawData = new Bframe(updateTime, 1);
  // The imu can take a bframe reference and fill out all the details
  imu.fillBframeWithRawData(*rawData);
  // Now we can link.
  // This will automatically drop the last element if the buffer overflows
  rawData->link(head);
  // Now we set the head to the pointer of the new rawData
  head = rawData;
  // To access the nth previous version we use the getPrevious(n) method
  Bframe* prev = head->getPrevious(1);

  // Check that the previous pointer actually exists
  // Since it uses time delta to calculate how much to integrate, the time current minus previous
  if(prev){
    // Since our quaternion library uses Radians we use the adafruit library to convert DPS to rads
    Vec3 angular_acceleration_rads = head->angular_velocity * SENSORS_DPS_TO_RADS;
    // A rotation in unit quaternion land is equivalent to double the rotation in real world
    Quat dqdt = (vehicle_orientation * 0.5f) * angular_acceleration_rads.euler_to_quat();
    Quat delq = dqdt * ((float)(head->timestamp - prev->timestamp) / 1e6);
    // Integratek
    vehicle_orientation = (vehicle_orientation + delq).normalize();
    Serial.print("The pitch of the vehicle is ");
    Serial.println(vehicle_orientation.quat_to_euler().y / SENSORS_DPS_TO_RADS);
  }

  delay(10);
}
