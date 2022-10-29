#include "sbus_controller.h"

bool SBusController::is_armed(uint16_t sbus_arm_switch) {
    if (sbus_arm_switch < kSBusMid)
        return false;
    return true;
}

uint8_t SBusController::manual_switch_state(uint16_t sbus_manual_switch) {
    if (sbus_manual_switch > (kSBusMax - kSBusThird)) {
        return 0;  // Manual
    } else if (sbus_manual_switch < kSBusThird) {
        return 2;  // Automatic Mode 2
    } else {
        return 1;  // Automatic Mode 1
    }
}

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

void SBusController::get_controls(float& roll_out, float& pitch_out, float& yaw_out, float& flap_out, float& motor_out) {
    // Order of max and min is dependent on if the SBus Signal is inverted or not
    int16_t roll_int = map(aileron(), kSBusEffMax, kSBusEffMin, -1000, 1000);
    int16_t pitch_int = map(elevator(), kSBusEffMin, kSBusEffMax, -1000, 1000);
    int16_t yaw_int = map(rudder(), kSBusEffMax, kSBusEffMin, -1000, 1000);
    int16_t flap_int = map(flap(), kSBusEffMin, kSBusEffMax, 0, 1000);
    int16_t motor_int = map(motor(), kSBusEffMax, kSBusEffMin, 0, 1000);
    
    roll_out = static_cast<float>(roll_int) / 1000.f;
    pitch_out = static_cast<float>(pitch_int) / 1000.f;
    yaw_out = static_cast<float>(yaw_int) / 1000.f;
    flap_out = static_cast<float>(flap_int) / 1000.f;
    motor_out = static_cast<float>(motor_int) / 1000.f;
}