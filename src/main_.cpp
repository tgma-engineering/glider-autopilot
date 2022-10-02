/*
 * Connect SBus:
 * SBus - GPIO25
 * and put 1kOhm between GPIO25 and GPIO26
 * It uses UART1
 * 
 * Servo Pins:
 * aileron_r - GPIO27
 * aileron_l - GPIO14
 * flap_r - GPIO12
 * flap_l - GPIO13
 * elevator - GPIO15
 * rudder - GPIO2
 * motor - GPIO4
 *
 * Default I2C lines for ESP32 are:
 * SCL - GPIO22
 * SDA - GPIO21
 * Those can be changed to any other pins via TwoWire Object
 * 
 * Connect GPS (Subject to change):
 * RX - GPIO35
 * TX - GPIO34
 * It uses UART2
 */
#if 0
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <SBUS2.h>
#include <ESP32Servo.h>

#define DEBUG 1

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

#if DEBUG
        for (uint8_t i = 0; i < 18; ++i) {
            Serial.print(sbus_channels[i]);
            Serial.print(" ");
        }
        Serial.print(sbus_fer);
        Serial.print(" ");
        Serial.print(failsave);
        Serial.println();
#endif
        
        test_servo.write(constrain(map(SBUS_MAX - sbus_channels[0], SBUS_MIN, SBUS_MAX, SERVO_PULS_MIN, SERVO_PULS_MAX), 1040, 1850));  // Quer, TIME: Mid-1470,Max-1850,Min-1040
        //test_servo.write(constrain(map(SBUS_MAX - sbus_channels[9], SBUS_MIN, SBUS_MAX, SERVO_PULS_MIN, SERVO_PULS_MAX), 1200, 2100));  // Klappen, TIME: Mid-2080,Max-2100,Min-1200
        //test_servo.write(constrain(map(sbus_channels[1], SBUS_MIN, SBUS_MAX, SERVO_PULS_MIN, SERVO_PULS_MAX), 863, 2272));  // Hoehe, TIME: Mid-1499,Max-2272,Min-863
        //test_servo.write(constrain(map(SBUS_MAX - sbus_channels[3], SBUS_MIN, SBUS_MAX, SERVO_PULS_MIN, SERVO_PULS_MAX), 862, 2080));  // Seite, TIME: Mid-1470,Max-2080,Min-862
        //test_servo.write(map(SBUS_MAX - sbus_channels[8], SBUS_MIN, SBUS_MAX, SERVO_PULS_MIN, SERVO_PULS_MAX));  // Motor, SBUS: 352-Off;1696-On

        esp_task_wdt_reset();  // Watchdog makes sure that SBUS is checked regularly
    }

    delay(50);
}
#endif