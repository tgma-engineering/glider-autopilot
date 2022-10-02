#include "sbus_controller.h"

SBusController::SBusController() : channels_{0} {
    fer_ = 0;
    failsave_ = false;
}

int8_t SBusController::setup() {
    SBUS2_Setup();
    return 0;
}

int8_t SBusController::loop(uint32_t dt) {
    if (SBUS_Ready()) {
        for (uint8_t i = 0; i < 18; ++i) {
            channels_[i] = SBUS2_get_servo_data(i);
        }
        uint16_t dummy1;
        bool dummy2;
        SBUS2_get_status(&dummy1, &dummy2, &failsave_);
        fer_ = SBUS_get_FER();

        return 0;
    }
    return -1;
}