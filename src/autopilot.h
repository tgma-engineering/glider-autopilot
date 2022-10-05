#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "sbus_controller.h"
#include "servo_controller.h"
#include "flight_controller.h"

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
        kManual = 0,  // Manual control
        kAuto1 = 1,   // Autopilot mode 1
        kAuto2 = 2    // Autopilot mode 2
    };

    Autopilot();
    void setup();
    void loop();

private:
    State state_;
    Mode mode_;
    uint32_t last_micros_;
    SBusController sbus_;
    ServoController servos_;
    FlightController fc_;
};

#endif  // AUTOPILOT_H_