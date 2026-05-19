#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SdFat.h>
#include <Adafruit_BMP280.h>
#include "SPI.h"
#include "Qmath.h"
#include "Vmath.h"
#include "quickMath.h"
#include "FS.h"
#include <ESP32Servo.h>
#include "DataLogger.h"

// setting up pins for the HAL
#define BUZZER_PIN 48

static const int PIN_SCK  = 5;
static const int PIN_MISO = 7;
static const int PIN_MOSI = 6;

static const int SD_CS  = 4;
static const int BMP_CS = 3;
static const int ICM_CS = 2;

bool SDC = true;

SPIClass spi;
SdFs sd;
FsFile logFile;
SdSpiConfig sdCfg(SD_CS, SHARED_SPI, SD_SCK_MHZ(4), &spi);


Adafruit_ICM20649 icm;
Adafruit_BMP280 bmp(BMP_CS, &spi);

int telemetryRate = 10; // telemetry rate in ms
unsigned long previousMillis = 0;

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
unsigned int t_apogee = 18000; // ms after launch to begin closout

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

class Telemetry{
  public:
    static const char* header(){
        return "System State, System State, Time since Launch(ms), omegaX, omegaY, omegaZ, aX, aY, aZ, vZ, vY, vZ, Pitch, Roll, Set Point(rads), EIntegral (rad-s), EDerivative (rad/s), Servo Deflection (rads), u(N-m), q(N/m^2), dt(s)";
    }
    int toCSV(char* buffer, size_t size) const {
        return snprintf(buffer, size,
             "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             (float)sys, (float)liftoffDetected, millis()-t_launch
             , angularVelocity.x, angularVelocity.y, angularVelocity.z
             , bodyAccel.x, bodyAccel.y, bodyAccel.z
             , inertialAccel.x, inertialAccel.y, inertialAccel.z
             , inertialVelocity.x, inertialVelocity.y, inertialVelocity.z
             , rotationQuat.q0, rotationQuat.q1, rotationQuat.q2, rotationQuat.q3
             , phi, roll, roll_target
             , e, Ie, De, delta, u, q
             , integrationMode, (float)integrateVelocity, t_buffer[0]-t_buffer[1]);
    }
};

Telemetry theTelemetry;

const unsigned long TEST_DURATION_MS = 5000;
const unsigned long SAMPLE_PERIOD_MS = 10;
const unsigned long FLUSH_PERIOD_MS  = 100; 

unsigned long startTime = 0;
unsigned long lastSample = 0;
unsigned long lastFlush = 0;

void setup_imu_sensor();
Vec3 calibrateGyro(int samples, int dt);
Vec3 calibrateAccelerometer(int samples, int dt);
unsigned long init_imu();
void reset_imu();
float noiseFilter(float sensor, float cutoff);
Vec3 vecFilter(Vec3 sensor, float cutoff);
unsigned long update_imu(unsigned long prevTime, Vec3 gyro, Vec3 accel, int mode, bool updateVelocity);
void compute_angles();
float servoAngle(float angle, float minAngle, float maxAngle);
void update_buffer(float src, float *buffer);

bool initSDLogFile() {
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ICM_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  if (!sd.begin(sdCfg)) {
    return false;
  }

  if (!logFile.open("/testfile.txt", O_WRONLY | O_APPEND | O_CREAT )) {
    return false;
  }

  logFile.println("Time_ms, System_State, Liftoff_Detected, Time_Since_Launch_ms, W_X, W_Y, W_Z, Body_A_X, Body_A_Y, Body_A_Z, Inertial_A_X, Inertial_A_Y, Inertial_A_Z, V_X, V_Y, V_Z, Rot_q0, Rot_q1, Rot_q2, Rot_q3, Phi_rad, Roll_rad, Roll_Target_rad, E_P, E_I, E_D, Fin_Deflection_rad, u, q_Pa, Integration_Mode, Integrate_Velocity, dt_us");
  logFile.flush();
  return !logFile.getWriteError();
}

bool log(unsigned long t) {
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ICM_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  // char buffer[256];
  // int n = theTelemetry.toCSV(buffer, sizeof(buffer));
  // if (n <= 0 || n >= (int)sizeof(buffer)) {
  //   return false;
  // }
  logFile.print(t);
  logFile.print(',');
  logFile.print(sys);
  logFile.print(',');
  logFile.print(liftoffDetected);
  logFile.print(',');
  logFile.print(millis()-t_launch);
  logFile.print(',');
  logFile.print(angularVelocity.x);
  logFile.print(',');
  logFile.print(angularVelocity.y);
  logFile.print(',');
  logFile.print(angularVelocity.z);
  logFile.print(',');
  logFile.print(bodyAccel.x);
  logFile.print(',');
  logFile.print(bodyAccel.y);
  logFile.print(',');
  logFile.print(bodyAccel.z);
  logFile.print(',');
  logFile.print(inertialAccel.x);
  logFile.print(',');
  logFile.print(inertialAccel.y);
  logFile.print(',');
  logFile.print(inertialAccel.z);
  logFile.print(',');
  logFile.print(inertialVelocity.x);
  logFile.print(',');
  logFile.print(inertialVelocity.y);
  logFile.print(',');
  logFile.print(inertialVelocity.z);
  logFile.print(',');
  logFile.print(rotationQuat.q0);
  logFile.print(',');
  logFile.print(rotationQuat.q1);
  logFile.print(',');
  logFile.print(rotationQuat.q2);
  logFile.print(',');
  logFile.print(rotationQuat.q3);
  logFile.print(',');
  logFile.print(phi);
  logFile.print(',');
  logFile.print(roll);
  logFile.print(',');
  logFile.print(roll_target);
  logFile.print(',');
  logFile.print(e);
  logFile.print(',');
  logFile.print(Ie);
  logFile.print(',');
  logFile.print(De);
  logFile.print(',');
  logFile.print(delta);
  logFile.print(',');
  logFile.print(u);
  logFile.print(',');
  logFile.print(q);
  logFile.print(',');
  logFile.print(integrationMode);
  logFile.print(',');
  logFile.print(integrateVelocity);
  logFile.print(',');
  logFile.print(t_buffer[0]-t_buffer[1]);

  // logFile.write((const uint8_t*)buffer, n);
  logFile.println();
  if(logFile.getWriteError()){
    // Serial.println("ERROR WRITING");
  }
  return !logFile.getWriteError();
}

void setup(void) {
  // replace with device objects
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  
  pinMode(SD_CS, OUTPUT);
  pinMode(BMP_CS, OUTPUT);
  pinMode(ICM_CS, OUTPUT);

  digitalWrite(SD_CS, HIGH);
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ICM_CS, HIGH);

  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  delay(20);

  //Wire.begin(SDA_PIN, SCL_PIN);
  // while (!Serial) delay(10); This will become problmeatic in flight
  Serial.println("Setting up ICM IMU");
  // Try to initialize!
  // if (!icm.begin_I2C()) {
  // if (!icm.begin_SPI(ICM_CS)) {
  while (!icm.begin_SPI(ICM_CS, &spi)) {
    Serial.println("ICM failed to init");
    delay(1000);
  }
  Serial.println("ICM20649 Found!");
  icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_361_4_HZ);
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_473_HZ);
  setup_imu_sensor();
  Serial.println("Setting up BMP Barometer");
  while(!bmp.begin()) {
    Serial.println("BMP failed to init");
    delay(1000);
  }
  Serial.println("BMP280 Found!");
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("Setting up ICM IMU");
  if (!initSDLogFile()) {
    Serial.println("SD init failed!");
    SDC = false;
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

  if ((sys != STANDBY) || groundTest == true || SDC == false){
    fin.write(servoAngle(60,-60,60));
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
      Serial.print(inertialAccel.x);
      Serial.print(",");
      Serial.print("A_Y:");
      Serial.print(inertialAccel.y);
      Serial.print(",");
      Serial.print("A_Z:");
      Serial.print(inertialAccel.z);
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
      telemetryRate = 10;
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
      telemetryRate = 10;
      // Activate Control loop
      // PID loop
      e = roll - roll_target; // in radians
      dt = (t_buffer[0] - t_buffer[1]) / 1000000; // in seconds
      Ie += e * dt;
      De = angularVelocity.x;
      //Ti = Kp / Ki;
      //Td = Kd / Kp;
      //u = u_buffer[0] + Kp * ((1 + dt / Ti + Td / dt) * e + (-1 - 2.0f * Td / dt) * e_buffer[0] + Td * e_buffer[1] / dt);
      u = Kp * e + Ki * Ie + Kd * De;
      A = 0.00032258f;
      q = 0.5f * (1.112f) * inertialVelocity.norm() * inertialVelocity.norm();
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
      De = angularVelocity.x;
      //Ti = Kp / Ki;
      //Td = Kd / Kp;
      //u = u_buffer[0] + Kp * ((1 + dt / Ti + Td / dt) * e + (-1 - 2.0f * Td / dt) * e_buffer[0] + Td * e_buffer[1] / dt);
      u = Kp * e + Ki * Ie + Kd * De;
      A = 0.0013f;
      q = 0.5f * (1.112f) * 200 * 200;
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
        delay(10);
      }
    break;
  }
  unsigned long now = millis();
  unsigned long currentMillis = now; // Get current time

  digitalWrite(SD_CS, HIGH);
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(ICM_CS, HIGH);
  
  if(SDC == true){

    if (currentMillis - previousMillis >= telemetryRate) {
      previousMillis = currentMillis; // Update the last execution time
        if (log(millis())){
          // Serial.println("Log success");
        }
    }

    lastSample = now;
    
    if (now - lastFlush >= FLUSH_PERIOD_MS) {
      logFile.flush();
    lastFlush = now;
    }
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
  bodyAccel = accel - rotationQuat.z_dir()*g;
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

void update_buffer(float src, float *buffer){
  for(int i = BUFFER_LEN-1; i>0; i--){
    buffer[i]=buffer[i-1];
  }
  buffer[0]=src;
}


