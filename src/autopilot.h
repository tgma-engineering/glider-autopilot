#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "sbus_controller.h"

class Autopilot {
public:
    static const uint8_t kWatchdogTimeout = 1;  // In Seconds

    enum State {
        kIdle,
        kArmFail,
        kDisarmed,
        kArmed
    };

    enum Mode {
        kManual,
        kAuto
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