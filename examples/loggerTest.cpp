#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

#include "DataLogger.h"
#include "Vmath.h"

static const int PIN_SCK  = 5;
static const int PIN_MISO = 7;
static const int PIN_MOSI = 6;
static const int PIN_CS   = 4;


SPIClass spi;
SdSpiConfig sdCfg(PIN_CS, SHARED_SPI, SD_SCK_MHZ(4), &spi);
SDLogger logger(sdCfg);

unsigned long startTime;
const unsigned long TEST_DURATION_US = 5e6;

unsigned long i = 0;

void setup() {
    Serial.begin(115200);
    while(!Serial){
      delay(10);
    }

    Serial.println("Starting SD Logger Test...");

    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);

    spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);

    if (!logger.begin()) {
        Serial.println("❌ SD init failed!");
        while (true);
    }

    Serial.println("✅ SD initialized");

    startTime = micros();
}

void loop() {
    i++;
    unsigned long now = micros();

    if (now - startTime > TEST_DURATION_US) {
        Serial.println("✅ Test complete");
        logger.end();
        while (true);
    }

    // ===== GENERATE TEST DATA =====

    // y = changing (sine wave)
    float y = sinf(now * 0.002f);

    // z = constant
    float z = 42.0f;

    Vec3 v(i, y, z);

    // ===== LOG DATA =====
    if (!logger.log("/testfile.txt", now, v)) {
        Serial.println("❌ Log failed");
    }
}