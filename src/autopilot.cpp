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

    // SBus
    Serial.println("Setup SBus ...");
    sbus_.setup();

    // Servos and motor
    Serial.println("Setup Servos ...");
    if (servos_.setup()) {
        while (true) {
            Serial.println("Error: Servo Setup failed");
            delay(1000);
        }
    }

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
            if (SBusController::is_armed(sbus_.arm_switch())) {
                state_ = kArmFail;
                Serial.println("Warning: System must start disarmed");
            } else {
                state_ = kDisarmed;
            }
        }
        if (state_ == kArmFail) {
            if (!SBusController::is_armed(sbus_.arm_switch())) {
                state_ = kDisarmed;
            }
        }
        if (state_ == kDisarmed) {
            if (SBusController::is_armed(sbus_.arm_switch())) {
                if (ServoController::is_motor_on(sbus_.motor())) {
                    state_ = kArmFail;
                    Serial.println("Warning: Motor cannot be on when arming");
                } else {
                    state_ = kArmed;
                }
            }
        }
        if (state_ == kArmed) {
            if (!SBusController::is_armed(sbus_.arm_switch())) {
                state_ = kDisarmed;
            }
        }
    }

    servos_.loop(dt);

    if (mode_ == kManual) {
        if (sbus_has_data) {
            bool set_motor = (state_ == kArmed && !sbus_.failsave());  // False will turn motor off
            servos_.set_from_sbus(sbus_, set_motor);
        }
    }  // Auto mode has yet to be implemented
}