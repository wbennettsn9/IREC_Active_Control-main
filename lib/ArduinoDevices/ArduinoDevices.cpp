#include "ArduinoDevices.h"

// SCL on icm
#define SENSOR_CLK 7
// AD0 on icm
#define SENSOR_MISO 6
// SDA on icm
#define SENSOR_MOSI 5
#define SENSOR_CS 4

// Check and CHANGE
#define ICM_INT_PIN 10

bool IMU_ICM20649::init(){
    if(!this->begin_SPI(SENSOR_CS, SENSOR_CLK, SENSOR_MISO, SENSOR_MOSI)){
        return false;
    }
    /*
    These are the ranges of the ICM20649
    The accelerometer and gyro output of ICM20649 is internally locked to units g and deg/s respectivly,
    So the higher the accel_range, the lower the resolution
    */

    pinMode(ICM_INT_PIN, INPUT);
    this->setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    Serial.print("Accelerometer range set to: ");
    switch (this->getAccelRange())
    {
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
    this->setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);
    Serial.print("Gyro range set to: ");
    switch (this->getGyroRange())
    {
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
    /*
    This sets the internal clock speed of the ICM20649
    The ICM20649 runs at this frequency given by the below equation
    When we _read() from the ICM20649 we are actually reading its last measured value
    The larger the divisor the slower the instrument runs
    */
    this->setAccelRateDivisor(0);
    uint16_t accel_divisor = this->getAccelRateDivisor();
    float accel_rate = 1125 / (1.0 + accel_divisor);

    Serial.print("Accelerometer data rate divisor set to: ");
    Serial.println(accel_divisor);
    Serial.print("Accelerometer data rate (Hz) is approximately: ");
    Serial.println(accel_rate);

    this->setGyroRateDivisor(0);
    uint8_t gyro_divisor = this->getGyroRateDivisor();
    float gyro_rate = 1100 / (1.0 + gyro_divisor);

    Serial.print("Gyro data rate divisor set to: ");
    Serial.println(gyro_divisor);
    Serial.print("Gyro data rate (Hz) is approximately: ");
    Serial.println(gyro_rate);
    Serial.println();

    Serial.println("SETUP COMPLETE");
    /* 
    Cutoff Frequencies for Accelerometer DLPF 
    ICM20X_ACCEL_FREQ_246_0_HZ = 246.0 Hz
    ICM20X_ACCEL_FREQ_111_4_HZ = 111.4 Hz
    ICM20X_ACCEL_FREQ_50_4_HZ = 50.4 Hz
    ICM20X_ACCEL_FREQ_23_9_HZ = 23.9 Hz
    ICM20X_ACCEL_FREQ_11_5_HZ = 11.5 Hz
    ICM20X_ACCEL_FREQ_5_7_HZ = 5.7 Hz
    ICM20X_ACCEL_FREQ_473_HZ = 473 Hz
    */
   icm20x_accel_cutoff_t accel_cutoff = ICM20X_ACCEL_FREQ_473_HZ;
   this->enableAccelDLPF(false, accel_cutoff);

    /*
    Cutoff Frequencies for Gyro DLPF 
    ICM20X_GYRO_FREQ_196_6_HZ = 196.6 Hz
    ICM20X_GYRO_FREQ_151_8_HZ = 151.8 Hz
    ICM20X_GYRO_FREQ_119_5_HZ = 119.5 Hz
    ICM20X_GYRO_FREQ_51_2_HZ = 51.2 Hz
    ICM20X_GYRO_FREQ_23_9_HZ = 23.9 Hz
    ICM20X_GYRO_FREQ_11_6_HZ = 11.6 Hz
    ICM20X_GYRO_FREQ_5_7_HZ = 5.7 Hz
    ICM20X_GYRO_FREQ_361_4_HZ = 361.4 Hz
    */
    icm20x_gyro_cutoff_t gyro_cutoff = ICM20X_GYRO_FREQ_23_9_HZ;
    this->enableGyrolDLPF(false, gyro_cutoff);

    if(!initInterrupt()){
        return false;
    };
    return true;
}

bool IMU_ICM20649::initInterrupt() {
    _setBank(0);

    // Enable raw data ready interrupt to propagate to interrupt pin 1
    Adafruit_BusIO_Register ICM20649_INT_ENABLE_1(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, 0x11);

    Adafruit_BusIO_RegisterBits ICM20649_RAW_DATA_0_RDY_EN(
        &ICM20649_INT_ENABLE_1, 1, 0);
    ICM20649_RAW_DATA_0_RDY_EN.write(true);

    // Configure INT1 behavior
    Adafruit_BusIO_Register ICM20649_INT_PIN_CFG(
        i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, 0x0F);

    // INT1 pin level held until interrupt status is cleared
    Adafruit_BusIO_RegisterBits ICM20649_INT1_LATCH__EN(
        &ICM20649_INT_PIN_CFG, 1, 5);
    ICM20649_INT1_LATCH__EN.write(true);

    // INT1 active low
    Adafruit_BusIO_RegisterBits ICM20649_INT1_ACTL(
        &ICM20649_INT_PIN_CFG, 1, 7);
    ICM20649_INT1_ACTL.write(true);

    // INT1 push-pull
    Adafruit_BusIO_RegisterBits ICM20649_INT1_OPEN(
        &ICM20649_INT_PIN_CFG, 1, 6);
    ICM20649_INT1_OPEN.write(false);

    pinMode(ICM_INT_PIN, INPUT);
}

unsigned long IMU_ICM20649::updateimu() {
    if (digitalRead(ICM_INT_PIN) != LOW) {
        return 0;
    }
    _read();
    return micros();
}

void IMU_ICM20649::fillBframeWithRawData(Bframe& frame){
    // The imu is not in line with the body frame
    // Y up in IMU frame is X up in body frame
    frame.acceleration = Vec3(accY, -accX, accZ);
    frame.angular_velocity = Vec3(gyroY, -gyroX, gyroZ);
}