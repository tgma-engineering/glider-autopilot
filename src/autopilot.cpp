#include "autopilot.h"

Autopilot::Autopilot() {
    state_ = kIdle;
    mode_ = kManual;
    last_micros_ = 0;
    fc_was_off_ = true;
}

void Autopilot::setup() {
    last_micros_ = micros();

    // Setup
    Serial.begin(115200);

    // Watchdog checks for infinite loops and resets MCU if it finds one
    Serial.println("Setup Setup Watchdog Timer ...");
    esp_err_t esp_err_init = esp_task_wdt_init(kSetupWatchdogTimeout, true);
    esp_err_t esp_err_add = esp_task_wdt_add(NULL);
    if (esp_err_init != ESP_OK || esp_err_add != ESP_OK) {
        while (true) {
            Serial.println("Error: Setup Watchdog Timer Setup failed");
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

    // Flight Controller
    Serial.println("Setup Flight Controller ...");
    if (fc_.setup()) {
        while (true) {
            Serial.println("Error: Flight Controller Setup failed");
            delay(1000);
        }
    }

    esp_task_wdt_reset();

    // Watchdog checks for infinite loops and resets MCU if it finds one
    Serial.println("Setup Watchdog Timer ...");
    esp_err_init = esp_task_wdt_init(kWatchdogTimeout, true);
    if (esp_err_init != ESP_OK) {
        while (true) {
            Serial.println("Error: Watchdog Timer Setup failed");
            delay(1000);
        }
    }

    //delay(1000);

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

#if WATCHDOG_OFF
    esp_task_wdt_reset();
#endif

    // SBus
    bool sbus_has_data = !sbus_.loop(dt);
    if (sbus_has_data)
        esp_task_wdt_reset();  // WDT makes sure that SBus is checked

    if (sbus_has_data) {
        mode_ = static_cast<Mode>(SBusController::manual_switch_state(sbus_.manual_switch()));

        // No safety features but the ability to quickly recover from system resets
        if ((state_ == kDisarmed || state_ == kIdle) && SBusController::is_armed(sbus_.arm_switch())) {
            state_ = kArmed;
            Serial.println("System is armed");
        } else if (state_ == kArmed && !SBusController::is_armed(sbus_.arm_switch())) {
            state_ = kDisarmed;
            Serial.println("System is disarmed");
        }
    }

    servos_.loop(dt);

    // Only keep track of attitude and position if in actual flight
    if (state_ == kArmed || mode_ != kManual) {
        if (fc_was_off_) {
            fc_.turn_on();  // Must be called if fc_.loop() is called for the first time in a while
            fc_was_off_ = false;
        }
        fc_.loop(dt);  // Keeps track of attitude, no controls
    } else {
        fc_was_off_ = true;
    }

    bool set_motor = (state_ == kArmed && !sbus_.failsave());  // False will turn motor off
    if (mode_ == kManual) {
        fc_.set_inactive();

        if (sbus_has_data) {
            servos_.set_from_sbus(sbus_, set_motor);

            // For tax purposes
            float roll, pitch, yaw, flap, motor;
            sbus_.get_controls(roll, pitch, yaw, flap, motor);
            fc_.set_input(roll, pitch, yaw, flap, motor);
        }
    } else {  // There is only one auto mode yet
        fc_.set_active();

        if (sbus_has_data) {
            float roll, pitch, yaw, flap, motor;
            sbus_.get_controls(roll, pitch, yaw, flap, motor);
            fc_.set_input(roll, pitch, yaw, flap, motor);
        }
        float roll, pitch, yaw, flap, motor;
        fc_.controls(roll, pitch, yaw, flap, motor, dt);
        if (!set_motor)
            motor = 0.f;
        servos_.set(roll, pitch, yaw, flap, motor);
    }
}