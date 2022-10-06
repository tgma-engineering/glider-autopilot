#include "flight_controller.h"

FlightController::FlightController() {
    is_active_ = false;
    input_roll_ = 0.f;
    input_pitch_ = 0.f;
    input_yaw_ = 0.f;
    input_flap_ = 0.f;
    input_motor_ = 0.f;
}

int8_t FlightController::setup() {
    if (imu_.setup()) {
        Serial.println("Error: IMU Setup failed");
        return -1;
    }
    return 0;
}

int8_t FlightController::loop(uint32_t dt) {
    imu_.loop(dt);  // Keep Track of attitude
    return 0;
}

void FlightController::set_active() {
    if (!is_active_) {
        is_active_ = true;
        target_attitude_ = imu_.attitude();  // Set current attitude as target
    }
}

void FlightController::set_inactive() {
    is_active_ = false;
}

void FlightController::set_input(float roll, float pitch, float yaw, float flap, float motor) {
    input_roll_ = roll;
    input_pitch_ = pitch;
    input_yaw_ = yaw;
    input_flap_ = flap;
    input_motor_ = motor;
}

void FlightController::controls(float& roll, float& pitch, float& yaw, float& flap, float& motor, uint32_t dt) {
    // Update target attitude using the inputs

    // Generate controls

    // Save controls in those
    roll = 0.f;
    pitch = 0.f;
    yaw = 0.f;
    flap = 0.f;
    motor = 0.f;
}