#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "SPI.h"
#include "Qmath.h"
#include "Vmath.h"
#include "quickMath.h"
#include "FS.h"
#include "SD.h"
#include <ESP32Servo.h>

// setting up pins for the HAL
#define BUZZER_PIN 48

#define ICM_SCK 7
#define ICM_MISO 6
#define ICM_MOSI 5
#define ICM_CS 4

#define SD_SCLK 39
#define SD_MISO 38
#define SD_MOSI 40
#define SD_CS 41
bool SDC = true;

SPIClass sd_spi(HSPI);
float telemetry[32];
int telemetryRate = 20; // telemetry rate in ms

// Global Vectors/Quaternions initialization
// Since these are global the current code does not use their histories in this way
Vec3 ig_cal; // Current calibrations
Vec3 ia_cal;
Vec3 angularVelocity; 
Vec3 bodyAccel;
Vec3 inertialAccel;
Vec3 inertialVelocity;
Vec3 bodyUpDirection = Vec3(1.0f,0.0f,0.0f);

Quat orientationQuat;
Quat rotationQuat;
float roll_target = 0;
float roll;
float eulerRoll;
float eulerPitch;
float eulerYaw;
float phi;
float g;

unsigned int dt = 1; // Main Loop Run Time, ms
unsigned int dt2 = 1; // Main Loop Run Time, ms
unsigned long lastUpdate; // timestamp of last control system update in ms

// Sensor Configuration
Adafruit_ICM20649 icm;
// Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro;
// uint16_t measurement_delay_us = 65535; // Delay between measurements for testing (IDK what this does but i think its important)
// Global sensor memory objects to use
sensors_event_t accel; 
sensors_event_t gyro;
sensors_event_t temp;

// Servo Configuration
Servo fin;
int servoPin = 19;

// Low Pass Filtering for gyro bias compensation
unsigned int stationaryTimer = 0;
float fc = 0.2f; // LPF corner frequency, Hz
float w_min = 0.04f; // cutoff speed for gyro, rad/s
float alpha; // Smoothing factor
Vec3 wp;
Vec3 gyroBias;
Vec3 oldGyroBias;
unsigned long test1;

// State Machine
enum State {
  BOOT,
  STANDBY,
  LIFTOFF,
  ACTIVE_CONTROL,
  GROUND_TEST,
  HWIL,
  CLOSEOUT
};
State sys = BOOT;
bool groundTest = false;

// Variables for adjusting IMU integration modes
int integrationMode = 1;
bool integrateVelocity = false;

#define BUFFER_LEN 5
float t_buffer[BUFFER_LEN];
float roll_buffer[BUFFER_LEN];
float u_buffer[BUFFER_LEN];
float e_buffer[BUFFER_LEN]; //euler?
float Ie_buffer[BUFFER_LEN]; // inertial orientation

// Liftoff Detection
float liftoffG = 3.0f; // acceleration to trigger liftoff
bool liftoffDetected = false; // true false detect if accel > maxAccel
unsigned long liftoffDetectionTime; // detected lift off time stamp to check for accidental drop
unsigned int accelerationTimer = 100; // minimum time that accel must be > maxAccel to enter Lift Off in ms

// Flight Timers
unsigned long t_launch; // timestamp of launch in ms
unsigned int t_burnout = 2000; // ms after to begin active control
unsigned int t_apogee = 13000; // ms after launch to begin closout

// PID/Control Loop Constants
float Kp = 0.5f;
float Ki = 0.5f;
float Kd = 0.05f;
float u = 0; // input
float e = 0; // error
float Ie = 0; // integral error
float De = 0; // derivative error
float q; // dynamic pressure
float delta = 0;
float max_dev; // max tilt angle
float A; // control surface area

void setup_imu_sensor();
Vec3 calibrateGyro(int samples, int dt);
Vec3 calibrateAccelerometer(int samples, int dt);
unsigned long init_imu();
void reset_imu();
float noiseFilter(float sensor, float cutoff);
Vec3 vecFilter(Vec3 sensor, float cutoff);
unsigned long update_imu(unsigned long prevTime, Vec3 gyro, Vec3 accel, int mode, bool updateVelocity);
void compute_angles() ;
float servoAngle(float angle, float minAngle, float maxAngle);
void initTelemetryFile();
void initTelemetryFile();
void writeTelemetry(float telemetry[], int size);
void getTelemetry();
void update_buffer(float src, float *buffer);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
void testFileIO(fs::FS &fs, const char * path);

void setup(void) {
  // replace with device objects
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  //Wire.begin(SDA_PIN, SCL_PIN);
  // while (!Serial) delay(10); This will become problmeatic in flight
    Serial.println("Adafruit ICM20649 test!");
    // Try to initialize!
      // if (!icm.begin_I2C()) {
      // if (!icm.begin_SPI(ICM_CS)) {
    if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
      Serial.println("Failed to find ICM20649 chip");
      while (1) {
      digitalWrite(BUZZER_PIN, HIGH); // Turn buzzer on
      delay(1000);                    // play for 1 second
      digitalWrite(BUZZER_PIN, LOW);  // Turn buzzer off
      delay(2000);                    // Wait 2 seconds
    }
  }

  Serial.println("ICM20649 Found!");
  digitalWrite(BUZZER_PIN, LOW);
  setup_imu_sensor();
  icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_361_4_HZ);
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_473_HZ);

  sd_spi.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sd_spi)) {
        Serial.println("Card Mount Failed");
        SDC = false;
        digitalWrite(BUZZER_PIN, HIGH);
    }
    uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    SDC = false;
    digitalWrite(BUZZER_PIN, HIGH);
  }

  if (SDC == true) {
    createDir(SD, "/flight_telemetry_files");
    initTelemetryFile();
    digitalWrite(BUZZER_PIN, LOW);
  }

  Serial.println("Starting Calibration Sequence");
  Serial.println("Starting Accelerometer Static Calibration");
  ia_cal = calibrateAccelerometer(5000,1);
  g = ia_cal.norm();
  Serial.print("g0 = ");
  Serial.println(g);
  Serial.println("Starting Gyro Static Calibration");
  ig_cal = calibrateGyro(5000, 1);
  Serial.println("Initialize IMU");

  // Servo Startup
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	fin.setPeriodHertz(50);    // standard 50 hz servo
	fin.attach(servoPin, 900, 2125); // attaches the servo on pin 1 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep

  Serial.print("Testing Servos\n");
  fin.write(servoAngle(0,-60,60));
  delay(1000);
  fin.write(servoAngle(60,-60,60));
  delay(1000);
  fin.write(servoAngle(-60,-60,60));
  delay(1000);
  fin.write(servoAngle(0,-60,60));
  delay(1000);

  delay(1000);
  SPI.setFrequency(7000000);
  //SPI.beginTransaction(SPISettings(7000000, MSBFIRST, SPI_MODE0));
  sys = STANDBY;

  if ((sys != STANDBY) || groundTest == true){
    digitalWrite(BUZZER_PIN, HIGH);
  }

  lastUpdate = init_imu();
}

void loop() {
  icm.getEvent(&accel, &gyro, &temp);
  /*icm_accel->getEvent(&accel);
  icm_gyro->getEvent(&gyro);*/
  Vec3 rawGyro;
  Vec3 calGyro;
  Vec3 bodyGyro;
  Vec3 rawAccel;
  Vec3 calAccel;
  Vec3 rawBodyAccel;

  rawGyro.x = gyro.gyro.x;
  rawGyro.y = gyro.gyro.y;
  rawGyro.z = gyro.gyro.z;

  calGyro.x = rawGyro.x - ig_cal.x;
  calGyro.y = rawGyro.y - ig_cal.y;
  calGyro.z = rawGyro.z - ig_cal.z;

  // Digital Low Pass filter used to eliminate bias error from gyroscope when the sensor is stationary
  // When the rocket on the pad we want to continously update gyro bias
  // Since gyro jumps around we use low pass filter to get this bias.
  // I should have a gyro bias object that is able to do this or look deeper in the imu code
  if (fabs(rawGyro.x) < w_min && fabs(rawGyro.y) < w_min && fabs(rawGyro.z) < w_min) {
    if ((millis() - stationaryTimer) > 500) {
      oldGyroBias.x = gyroBias.x;
      oldGyroBias.y = gyroBias.y;
      oldGyroBias.z = gyroBias.z;
      alpha = 2*PI*fc*(float)dt2 / 1000000;
      Vec3 tempBias = calGyro - oldGyroBias;
      tempBias = tempBias * alpha;
      gyroBias = oldGyroBias + tempBias;
      calGyro = calGyro - gyroBias;
    }
  }
  else
  {
    stationaryTimer = millis();
    // Serial.println("IMU Moving");
  }

  rawAccel.x = accel.acceleration.x;
  rawAccel.y = accel.acceleration.y;
  rawAccel.z = accel.acceleration.z;

  // Transform IMU frame to body frame. Y up in IMU frame is X up in body frame
  // I can give raw Bframe or I can give calibrated Bframes

  bodyGyro.x = calGyro.y;
  bodyGyro.y = -calGyro.x;
  bodyGyro.z = calGyro.z;

  rawBodyAccel.x = rawAccel.y;
  rawBodyAccel.y = -rawAccel.x;
  rawBodyAccel.z = rawAccel.z;
  angularVelocity = bodyGyro;

  float oldLastUpdate = lastUpdate;
  lastUpdate = update_imu(lastUpdate, bodyGyro, rawBodyAccel, integrationMode, integrateVelocity);
  // There is always a risk of buffer overflow.
  // -> send to the bufferMill but keep x active frames
  
  update_buffer(micros(), t_buffer);
  update_buffer(roll, roll_buffer);
  update_buffer(u, u_buffer); // u is control input
  update_buffer(e, e_buffer); // error history // This can be bframe
  update_buffer(Ie, Ie_buffer); // integral error // This can be a bframe
  
  dt2 = micros() - oldLastUpdate;

  switch (sys){
    case STANDBY:
      // IMU Integrator Mode
      integrationMode = 1;
      integrateVelocity = false;
      /*Serial.print("BodyAccel/g:");
      Serial.print(bodyAccel.x / g);
      Serial.print(",");
      Serial.print("V:");
      Serial.print(inertialVelocity.norm());
      Serial.print(",");
      Serial.print("A_X:");
      Serial.print(bodyAccel.x);
      Serial.print(",");
      Serial.print("A_Y:");
      Serial.print(bodyAccel.y);
      Serial.print(",");
      Serial.print("A_Z:");
      Serial.print(bodyAccel.z);
      Serial.println();*/

      // Liftoff Detection Algorithm
      // Check for acceleration exceeding limit
      if ((bodyAccel.x / g) > liftoffG && (liftoffDetected == false)){
        liftoffDetected = true;
        liftoffDetectionTime = millis();
        Serial.println("Liftoff acceleration detected");
        // reset_imu(); // assumes liftoff orientation is "up"
      }
      // check for acceleration dropping below limit after liftoff timer activates
      if ((liftoffDetected == true) && ((bodyAccel.x / g) < liftoffG)){
        liftoffDetected = false;
        Serial.println("Liftoff detection interrupted, resetting timer");
        liftoffDetectionTime = millis();
      }
      // confirm liftoff and switching to liftoff mode
      if((liftoffDetected == true) && (millis() - liftoffDetectionTime > accelerationTimer)){
        Serial.println("Liftoff Confirmed");
        sys=LIFTOFF;
        t_launch=millis();
        integrationMode = 2; // start integrating gyroscope
        integrateVelocity = true; // start integrating velocity
        break;
      }
    break;
    case LIFTOFF:
      integrationMode = 2;
      integrateVelocity = true;
      // Check if it is time to activate control system, after motor burnout
      if(millis()-t_launch>t_burnout){
        sys = groundTest == true ? GROUND_TEST : ACTIVE_CONTROL;
        roll_target = roll;
        Serial.println("Burnout timer passed");
        break;
      }
    break;
    case ACTIVE_CONTROL:
      integrationMode = 2;
      integrateVelocity = true;
      // Activate Control loop
      // PID loop
      e = roll - roll_target; // in radians
      dt = (t_buffer[0] - t_buffer[1]) / 1000000; // in seconds
      Ie += e * dt;
      De = (e - e_buffer[0]) / dt;
      //Ti = Kp / Ki;
      //Td = Kd / Kp;
      //u = u_buffer[0] + Kp * ((1 + dt / Ti + Td / dt) * e + (-1 - 2.0f * Td / dt) * e_buffer[0] + Td * e_buffer[1] / dt);
      u = Kp * e + Ki * Ie + Kd * De;
      A = 0.00032258f;
      q = 0.5f * (1.112f) * inertialVelocity.norm();
      delta = 2.0f * u / (0.0892f * q * A * 6.66f);
      Ie = abs(delta * 57.2958f) > 20 ? Ie_buffer[0] : Ie;

      fin.write(servoAngle(delta * 57.2958f,-20,20));
      
      // check saftey parameters to closout
      max_dev = 30.0f * 0.0174533f;
      if(phi > max_dev){
        Serial.println("UNSAFE ANGLE DETECTED, DISABLING ROLL CONTROL");
        //fin.write(servoAngle(0.0f,-30,30));
        //sys = CLOSEOUT;
      }
      if(millis()-t_launch>t_apogee){
        sys = CLOSEOUT;
        break;
      }
    break;

    case GROUND_TEST:
      // Activate Ground Test Loop
      // PID loop
      digitalWrite(BUZZER_PIN, HIGH);
      integrationMode = 2;
      integrateVelocity = true;
      e = roll - roll_target; // in radians
      dt = (t_buffer[0] - t_buffer[1]) / 1000000; // in seconds
      Ie += e * dt;
      De = (e - e_buffer[0]) / dt;
      //Ti = Kp / Ki;
      //Td = Kd / Kp;
      //u = u_buffer[0] + Kp * ((1 + dt / Ti + Td / dt) * e + (-1 - 2.0f * Td / dt) * e_buffer[0] + Td * e_buffer[1] / dt);
      u = Kp * e + Ki * Ie + Kd * De;
      A = 0.00032258f;
      q = 0.5f * (1.112f) * 50 * 50;
      delta = 2.0f * u / (0.0892f * q * A * 6.66f);
      Ie = abs(delta * 57.2958f) > 20 ? Ie_buffer[0] : Ie;

      fin.write(servoAngle(delta * 57.2958f,-20,20));

      Serial.print("q0:");
      Serial.print(orientationQuat.q0);
      Serial.print(",");
      Serial.print("q1:");
      Serial.print(orientationQuat.q1);
      Serial.print(",");
      Serial.print("q2:");
      Serial.print(orientationQuat.q2);
      Serial.print(",");
      Serial.print("q3:");
      Serial.print(orientationQuat.q3);
      Serial.print(",");
      Serial.print("phi:");
      Serial.print(phi);
      Serial.print(",");
      Serial.print("V:");
      Serial.print(inertialVelocity.norm());
      Serial.println();
      
      // check saftey parameters to closout
      max_dev = 30.0f * 0.0174533f;
      if(phi > max_dev){
      Serial.println("UNSAFE ANGLE DETECTED, DISABLING ROLL CONTROL");
      //sys = CLOSEOUT;
      }

      if(millis()-t_launch>t_apogee){
        // sys = CLOSEOUT;
        break;
      }
    break;

    case HWIL:
    {
    break;
    }

    case CLOSEOUT:
      Serial.println("Closeout complete!");
      while(1){
        digitalWrite(BUZZER_PIN, HIGH); // Turn buzzer on
        delay(2000);                    // play for 2 seconds
        digitalWrite(BUZZER_PIN, LOW);  // Turn buzzer off
        delay(2000);                    // Wait 2 seconds
        delay(10);
      }
    break;
  }
  
  if(SDC == true && millis()%(telemetryRate)<5){
    int size = sizeof(telemetry) / sizeof(telemetry[0]);
    getTelemetry();
    writeTelemetry(telemetry, size);
  }
}

// Configure the Sensors and print out the current settings
void setup_imu_sensor(){
  icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
    case ICM20649_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
    break;
    case ICM20649_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
    break;
    case ICM20649_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
    break;
    case ICM20649_ACCEL_RANGE_30_G:
      Serial.println("+-30G");
    break;
  }
  icm.setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
    case ICM20649_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
    break;
    case ICM20649_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
    break;
    case ICM20649_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
    break;
    case ICM20649_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }
  icm.setAccelRateDivisor(0);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);
  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);
  icm.setGyroRateDivisor(0);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);
  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);
}

// Calibrate a sensor statically and returns the offset vector
Vec3 calibrateGyro(int samples, int dt){
  Vec3 sensor_cal;
  sensor_cal.x = 0;
  sensor_cal.y = 0;
  sensor_cal.z = 0;
  for(int i = 0; i<samples; i++){
    icm.getEvent(&accel, &gyro, &temp);
    sensor_cal.x+=(double)(gyro.gyro.x/samples);
    sensor_cal.y+=(double)(gyro.gyro.y/samples);
    sensor_cal.z+=(double)(gyro.gyro.z/samples);
    delay(dt);
  }

  Serial.println("Gyro Calibration complete");
  Serial.print("Offset : ");
  Serial.print(sensor_cal.x);
  Serial.print(",");
  Serial.print(sensor_cal.y);
  Serial.print(",");
  Serial.print(sensor_cal.z);
  Serial.println();
  return sensor_cal;
}

Vec3 calibrateAccelerometer(int samples, int dt){
  Vec3 sensor_cal;
  sensor_cal.x = 0;
  sensor_cal.y = 0;
  sensor_cal.z = 0;
  for(int i = 0; i<samples; i++){
    icm.getEvent(&accel, &gyro, &temp);
    sensor_cal.x+=(double)(accel.acceleration.x/samples);
    sensor_cal.y+=(double)(accel.acceleration.y/samples);
    sensor_cal.z+=(double)(accel.acceleration.z/samples);
    delay(dt);
  }

  Serial.println("Accelerometer Calibration complete");
  Serial.print("Offset : ");
  Serial.print(sensor_cal.x);
  Serial.print(",");
  Serial.print(sensor_cal.y);
  Serial.print(",");
  Serial.print(sensor_cal.z);
  Serial.println();
  return sensor_cal;
}

// Sets the IMU to default values
unsigned long init_imu(){
	rotationQuat = Quat(1.0f,0.0f,0.0f,0.0f);
  orientationQuat = Quat(0.0f,1.0f,0.0f,0.0f);
  inertialVelocity = Vec3(0.0f,0.0f,0.0f);
  roll = 0.0f;
  return micros();
}

// Sets the IMU to default values
void reset_imu(){
	rotationQuat = Quat(1.0f,0.0f,0.0f,0.0f);
  orientationQuat = Quat(0.0f,1.0f,0.0f,0.0f);
  inertialVelocity = Vec3(0.0f,0.0f,0.0f);
  roll = 0.0f;
}

float noiseFilter(float sensor, float cutoff) {
  return fabs(sensor) > cutoff ? sensor : 0.0f;
}

Vec3 vecFilter(Vec3 sensor, float cutoff) {
  Vec3 filtered;
  filtered.x = fabs(sensor.x) > cutoff ? sensor.x : 0.0f;
  filtered.y = fabs(sensor.y) > cutoff ? sensor.y : 0.0f;
  filtered.z = fabs(sensor.z) > cutoff ? sensor.z : 0.0f;
  return filtered;
}

unsigned long update_imu(unsigned long prevTime, Vec3 gyro, Vec3 accel, int mode, bool updateVelocity){ // 3 modes: 0 = no update, 1 = accelerometer integration, 2 = gyro integration
  int dt = micros() - prevTime;
  Vec3 body_up = bodyUpDirection;

  Vec3 rawAccel; // = accel;
  rawAccel.x = ia_cal.y;
  rawAccel.y = -ia_cal.x;
  rawAccel.z = ia_cal.z;
	
  switch (mode) {
    // No update
    case 0:
      break;
    // Accelerometer Integration
    case 1:
    {
      // Acceleration vector is in direction of gravity while on pad
      Vec3 body_grav = rawAccel.normalize();
      // The acceleration of gravity 
      Vec3 inertial_grav = Vec3(0.0f,0.0f,1.0f);
      Vec3 rotation_axis = cross(body_grav, inertial_grav).normalize();
      float rotation_cosine = dot(body_grav, inertial_grav);
      float half_cosine = 1.0f / invSqrt((1 + rotation_cosine) / 2.0f);
      float half_sine = 1.0f / invSqrt((1 - rotation_cosine) / 2.0f);
      rotationQuat = Quat(half_cosine, half_sine*rotation_axis.x, half_sine*rotation_axis.y, half_sine*rotation_axis.z);
      break;
    }
    // Gyroscope Integration
    case 2:
    {
      // Rate of change of Quaternion from gyroscope
      Quat ang_vel = gyro.euler_to_quat();
      Quat qDot = (rotationQuat * 0.5f) * ang_vel;
      // Integrate rate of change of Quaternion to yield new orientation Quaternion
      Quat qDel = qDot * ((float)dt / 1000000);
      rotationQuat = rotationQuat + qDel;
      rotationQuat = rotationQuat.normalize();
      roll += gyro.x * (float)dt / 1000000;
      break;
    }
    case 3:
      break;
  }
  
  orientationQuat = rotatePassive(rotationQuat,body_up.euler_to_quat());
  compute_angles();
  bodyAccel = accel - Vec3(0,0,rotationQuat.quat_to_euler().z*g);
  Quat accel_global = rotatePassive(rotationQuat,bodyAccel.euler_to_quat());
  inertialAccel = Vec3(accel_global.q1,accel_global.q2,accel_global.q3);
  if (updateVelocity) {
    inertialVelocity = inertialVelocity + (inertialAccel * ((float)dt / 1000000));
  }

  return prevTime + dt;
}

void compute_angles() 
{
	eulerRoll = atan2f(rotationQuat.q0*rotationQuat.q1 + rotationQuat.q2*rotationQuat.q3, 0.5f - rotationQuat.q1*rotationQuat.q1 - rotationQuat.q2*rotationQuat.q2);
	eulerPitch = asinf(-2.0f * (rotationQuat.q1*rotationQuat.q3 - rotationQuat.q0*rotationQuat.q2));
	eulerYaw = atan2f(rotationQuat.q1*rotationQuat.q2 + rotationQuat.q0*rotationQuat.q3, 0.5f - rotationQuat.q2*rotationQuat.q2 - rotationQuat.q3*rotationQuat.q3);
  phi = acos(orientationQuat.q3);
}

float servoAngle(float angle, float minAngle, float maxAngle) {
  float targetAngle = constrain(angle, minAngle, maxAngle);
  targetAngle *= 1.5; // Scaling factor for servo travel of ±60 degrees for commanded input of ±90 degrees
  targetAngle += 90; // Shifts center from 0 to 90
  return targetAngle;
}

void initTelemetryFile() {
  if (!SD.exists("/flight_telemetry_files/telemetry.txt")) {
    File file = SD.open("/flight_telemetry_files/telemetry.txt", FILE_WRITE); // Open in write mode to create/overwrite
    Serial.print("Creating new telemetry file and adding headers...\n");
    if(file) {
      file.print("Time (ms), System State, Liftoff Detected, Time Since Launch (ms), omega_x, omega_y, omega_z, a_x (body), a_y (body), a_z (body), a_x (inertial), a_y (inertial), a_z (inertial), V_x, V_y, V_z, q0, q1, q2, q3, Tilt Angle, Roll (rads), Set Point (rads), PID Error (rads), Integral Error (rad-s), Derivative Error (rad/s), Servo Deflection (rads), u (N-m), q (N/m^2), IMU Integration Mode, dt (μs)\n"); // Write CSV headers
      file.close();
      Serial.print("Headers written\n");
    } 
    else {
      Serial.print("Failed to create file\n");
    }
  }
  else {
    Serial.print("Telemetry file already exists, creating new flight\n");
    appendFile(SD, "/flight_telemetry_files/telemetry.txt", "======================================== NEW FLIGHT ========================================\n");
    String telem = "Omega_x CALIBRATION: " + String(ig_cal.x) + ", Omega_y CALIBRATION: " + String(ig_cal.y) + ", Omega_z CALIBRATION: " + String(ig_cal.z) + ", A_x CALIBRATION: " + String(ia_cal.x) + ", A_y CALIBRATION, " + String(ia_cal.y) +  + ", A_z CALIBRATION: " + String(ia_cal.z) + "\n";
    appendFile(SD, "/flight_telemetry_files/telemetry.txt", telem.c_str());
    appendFile(SD, "/flight_telemetry_files/telemetry.txt", "Time (ms), System State, Liftoff Detected, Time Since Launch (ms), omega_x, omega_y, omega_z, a_x (body), a_y (body), a_z (body), a_x (inertial), a_y (inertial), a_z (inertial), V_x, V_y, V_z, q0, q1, q2, q3, Tilt Angle, Roll (rads), Set Point (rads), PID Error (rads), Integral Error (rad-s), Derivative Error (rad/s), Servo Deflection (rads), u (N-m), q (N/m^2), IMU Integration Mode, dt (μs)\n"); // Write CSV headers
  }
}

void writeTelemetry(float telemetry[], int size) {
  String telem = "";
  for (int i = 0; i < size - 1; i++) {
    telem = telem + String(telemetry[i]) + ", ";
  }
  telem = telem + String(telemetry[size - 1]);
  telem = telem + "\n";
  appendFile(SD, "/flight_telemetry_files/telemetry.txt", telem.c_str());
  //appendFile(SD, "/telemetry/telemetry.txt", "\n");
}

void getTelemetry(){
  telemetry[0] = millis();
  switch (sys){
    case STANDBY:
      telemetry[1] = 0;
    break;
    case LIFTOFF:
      telemetry[1] = 1;
    break;
    case ACTIVE_CONTROL:
      telemetry[1] = 2;
    break;
    case GROUND_TEST:
      telemetry[1] = 3;
    break;
    case HWIL:
      telemetry[1] = 5;
    break;
    case CLOSEOUT:
      telemetry[1] = 6;
    break;
  }
  telemetry[2] = (float)liftoffDetected;
  telemetry[3] = millis()-t_launch;
  telemetry[4] = angularVelocity.x;
  telemetry[5] = angularVelocity.y;
  telemetry[6] = angularVelocity.z;
  telemetry[7] = bodyAccel.x;
  telemetry[8] = bodyAccel.y;
  telemetry[9] = bodyAccel.z;
  telemetry[10] = inertialAccel.x;
  telemetry[11] = inertialAccel.y;
  telemetry[12] = inertialAccel.z;
  telemetry[13] = inertialVelocity.x;
  telemetry[14] = inertialVelocity.y;
  telemetry[15] = inertialVelocity.z;
  telemetry[16] = rotationQuat.q0;
  telemetry[17] = rotationQuat.q1;
  telemetry[18] = rotationQuat.q2;
  telemetry[19] = rotationQuat.q3;
  telemetry[20] = phi;
  telemetry[21] = roll;
  telemetry[22] = roll_target;
  telemetry[23] = e;
  telemetry[24] = Ie;
  telemetry[25] = De;
  telemetry[26] = delta;
  telemetry[27] = u;
  telemetry[28] = q;
  telemetry[29] = integrationMode;
  telemetry[30] = (float)integrateVelocity;
  telemetry[31] = t_buffer[0] - t_buffer[1];
}

void update_buffer(float src, float *buffer){
  for(int i = BUFFER_LEN-1; i>0; i--){
    buffer[i]=buffer[i-1];
  }
  buffer[0]=src;
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    //Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    //Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  //Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    //Serial.println("Failed to open file for reading");
    return;
  }

  //Serial.print("Read from file: ");
  while(file.available()){
    //Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  //Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    //Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    //Serial.println("File written");
  } else {
    //Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  //Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    SDC = false;
    //Serial.println("Failed to open file for appending");
    //return;
  }
  if(file.print(message)){
      //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
    SDC = false;
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    //Serial.println("File renamed");
  } else {
    //Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  //Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    //Serial.println("File deleted");
  } else {
   // Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}
