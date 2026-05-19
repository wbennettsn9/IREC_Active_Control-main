#include <unity.h>
#include <Arduino.h>
#include "ArduinoDevices.h"

IMU_ICM20649 imu;

void test_imu_interrupt_generates_data_ready_events() {
    TEST_ASSERT_TRUE_MESSAGE(imu.init(), "IMU init failed");

    const unsigned long timeout_ms = 1000;
    const int min_events = 3;
    int events = 0;
    unsigned long start = millis();

    while ((millis() - start) < timeout_ms) {
        if (imu.updateimu() != 0) {
            events++;
            if (events >= min_events) {
                break;
            }
        }
        delay(1);
    }

    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(min_events, events,
                                             "Interrupt pin did not trigger expected data-ready events");
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_imu_interrupt_generates_data_ready_events);
    UNITY_END();
}

void loop() {}
