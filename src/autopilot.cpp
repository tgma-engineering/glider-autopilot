#include "autopilot.h"

Autopilot::Autopilot() {
    state_ = kIdle;
    mode_ = kManual;
    last_micros_ = 0;
}

void Autopilot::setup() {
    last_micros_ = micros();

    // Setup
    Serial.begin(115200);

    // Watchdog checks for infinite loops and resets MCU if it finds one
    Serial.println("Setup Watchdog Timer ...");
    esp_err_t esp_err_init = esp_task_wdt_init(kWatchdogTimeout, true);
    esp_err_t esp_err_add = esp_task_wdt_add(NULL);
    if (esp_err_init != ESP_OK || esp_err_add != ESP_OK) {
        while (true) {
            Serial.println("Error: Watchdog Timer Setup failed");
            delay(1000);
        }
    }

    // Sbus
    Serial.println("Setup SBUS ...");
    sbus_.setup();

    // Timing
    uint32_t curr_micros = micros();
    Serial.print("Setup Time: ");
    Serial.print(curr_micros - last_micros_);
    Serial.println("us");
    last_micros_ = curr_micros;
}

void Autopilot::loop() {
    // Timing
    uint32_t curr_micros = micros();
    uint32_t dt = 0;
    if (curr_micros >= last_micros_) {
        dt = curr_micros - last_micros_;
    } else {  // micros() overflow
        dt = 0xFFFFFFFFul - last_micros_ + curr_micros;
    }
    last_micros_ = curr_micros;

    // SBus
    bool sbus_has_data = !sbus_.loop(dt);
    if (sbus_has_data)
        esp_task_wdt_reset();  // WDT makes sure that SBus is checked

    if (sbus_has_data) {
        if (state_ == kIdle) {
            if (sbus_.arm_switch()) {
                state_ = kArmFail;
                Serial.println("Error: System must start disarmed");
            } else {
                state_ = kDisarmed;
            }
        }
        if (state_ == kArmFail) {
            if (!sbus_.arm_switch()) {
                state_ = kDisarmed;
            }
        }
        if (state_ == kDisarmed) {
            if (sbus_.arm_switch()) {
                // TODO: Make function in ServoController that replace checks like that
                if (sbus_.motor() > sbus_.kSBusMid) {  // Motor off
                    state_ = kArmed;
                } else {  // Motor on
                    state_ = kArmFail;
                    Serial.println("Error: Motor cannot be on when arming");
                }
            }
        }
        if (state_ == kArmed) {
            if (!sbus_.arm_switch()) {
                state_ = kDisarmed;
            }
        }
    }

    if (mode_ == kManual && sbus_has_data) {
        // Set servos (but not motor) from SBus
        if (state_ == kArmed) {
            // Set motor from SBus
        } else {
            // Turn off motor if it is on
        }
    }  // Auto mode is yet to be implemented
}