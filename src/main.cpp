/*
 * Connect SBUS line to IO25 and put 1kOhm between IO25 and IO26
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <SBUS2.h>
#include <ESP32Servo.h>

#define WATCHDOG_TIMEOUT 1  // 1 second
#define SBUS_MIN 0  // 12-Bit Signal
#define SBUS_MAX 2047
#define SERVO_PULS_MIN 544  // In microseconds
#define SERVO_PULS_MAX 2400

// Pins
#define SERVO_PIN 12

int16_t sbus_channels[18] = {0};
uint8_t sbus_fer = 0;  // Frame Error Rate: 0 is good, 100 is bad
bool failsave = false;

Servo test_servo;

void setup()
{
    Serial.begin(115200);

    // Watchdog checks for infinite loops and resets mcu if it finds one
    Serial.println("Setup Watchdog Timer ...");
    esp_err_t esp_err_init = esp_task_wdt_init(WATCHDOG_TIMEOUT, true);
    esp_err_t esp_err_add = esp_task_wdt_add(NULL);
    if (esp_err_init != ESP_OK || esp_err_add != ESP_OK) {
        while (true) {
            Serial.println("Error: Watchdog Timer Setup failed");
            delay(1000);
        }
    }

    Serial.println("Setup SBUS ...");
    SBUS2_Setup();

    Serial.println("Setup Servos ...");
    if (!test_servo.attach(SERVO_PIN)) {
        while (true) {
            Serial.println("Error: No free channel on Servo GPIO");
            delay(1000);
        }
    }
}

void loop()
{
    if (SBUS_Ready()) {
        for (uint8_t i = 0; i < 18; ++i) {
            sbus_channels[i] = SBUS2_get_servo_data(i);
        }
        uint16_t dummy1;
        bool dummy2;
        SBUS2_get_status(&dummy1, &dummy2, &failsave);
        sbus_fer = SBUS_get_FER();  // Frame Error Rate

        for (uint8_t i = 0; i < 18; ++i) {
            Serial.print(sbus_channels[i]);
            Serial.print(" ");
        }
        Serial.print(sbus_fer);
        Serial.print(" ");
        Serial.print(failsave);
        Serial.println();

        test_servo.write(map(sbus_channels[0], SBUS_MIN, SBUS_MAX, SERVO_PULS_MIN, SERVO_PULS_MAX));

        esp_task_wdt_reset();  // Watchdog makes sure that SBUS is checked regularly
    }

    delay(50);
}