#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "sbus_controller.h"

class Autopilot {
public:
    static const uint8_t kWatchdogTimeout = 1;  // In Seconds

    enum State {
        kIdle,      // Wait for SBus
        kArmFail,   // Arm failed somehow, disarm to continue
        kDisarmed,  // Default flight-ready state
        kArmed      // The only state in which motor is enabled
    };

    enum Mode {
        kManual,  // Manual control
        kAuto     // Autopilot mode
    };

    Autopilot();
    void setup();
    void loop();

private:
    State state_;
    Mode mode_;
    uint32_t last_micros_;
    SBusController sbus_;
};

#endif  // AUTOPILOT_H_