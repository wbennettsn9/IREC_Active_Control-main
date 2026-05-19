#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20649.h>
#include "Vmath.h"
#include "FrameBuffers.h"

class IMU_ICM20649 : public Adafruit_ICM20649{
    public:
        /*
        @brief Initialize the sensors
        Call at the beginning of the program to register the sensors to the I2C bus
        Set up the configurations for the sensor
        */
        bool init();

        /*
        @brief Read the latest sensor values 
        */
        unsigned long updateimu();

        /*
        @brief Get the most recent data buffer
        Does not resample, but instead uses last sample.
        */
        void fillBframeWithRawData(Bframe& frame);
    private:
        bool initInterrupt();
};