#include <Arduino.h>
#include <SBUS2.h>
#include <ESP32Servo.h>

uint8_t sbus_fer = 0;
int16_t sbus_channels[18] = {0};

void setup()
{
    Serial.begin(115200);

    Serial.println("Setup SBUS ...");
    SBUS2_Setup();
}

void loop()
{
    if (SBUS_Ready()) {
        for (uint8_t i = 0; i < 18; ++i) {
            sbus_channels[i] = SBUS2_get_servo_data(i);
        }
        sbus_fer = SBUS_get_FER();  // Frame Error Rate
    }

    for (uint8_t i = 0; i < 18; ++i) {
        Serial.print(sbus_channels[i]);
        Serial.print(" ");
    }
    Serial.print(sbus_fer);
    Serial.println();

    delay(50);
}