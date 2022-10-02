#include "servo_controller.h"

uint16_t ServoController::sbus_to_time(uint16_t sbus_channel, bool invert) {
    uint16_t channel = sbus_channel;
    if (invert)
        channel = kSBusMax - sbus_channel;
    return map(channel, kSBusMin, kSBusMax, kPulsMin, kPulsMax);
}

// Computes if motor speed is above or below midpoint given the SBus input
bool ServoController::is_motor_on(uint16_t sbus_motor) {
    uint16_t motor_time = constrain(sbus_to_time(sbus_motor, kMotorSBusInv), kMotorMin, kMotorMax);
    if (motor_time > 1472)
        return true;
    return false;
}

int8_t ServoController::setup() {
    bool error = false;
    if (!aileron_l_.attach(kAileronLGpio))
        error = true;
    if (!aileron_r_.attach(kAileronRGpio))
        error = true;
    if (!elevator_.attach(kElevatorGpio))
        error = true;
    if (!rudder_.attach(kRudderGpio))
        error = true;
    if (!flap_l_.attach(kFlapLGpio))
        error = true;
    if (!flap_r_.attach(kFlapRGpio))
        error = true;
    if (!motor_.attach(kMotorGpio))
        error = true;
    
    if (error)
        return -1;
    return 0;
}

int8_t ServoController::loop(uint32_t dt) {
    return 0;
}

void ServoController::set_from_sbus(const SBusController& sbus, bool write_motor) {
    uint16_t aileron_time = constrain(sbus_to_time(sbus.aileron(), kAileronSBusInv), kAileronMin, kAileronMax);
    aileron_l_.write(aileron_time);
    aileron_r_.write(aileron_time);
    elevator_.write(constrain(sbus_to_time(sbus.elevator(), kElevatorSBusInv), kElevatorMin, kElevatorMax));
    rudder_.write(constrain(sbus_to_time(sbus.rudder(), kRudderSBusInv), kRudderMin, kRudderMax));
    uint16_t flap_time = constrain(sbus_to_time(sbus.flap(), kFlapSBusInv), kFlapMin, kFlapMax);
    flap_l_.write(flap_time);
    flap_r_.write(flap_time);
    if (write_motor) {
        motor_.write(constrain(sbus_to_time(sbus.motor(), kMotorSBusInv), kMotorMin, kMotorMax));
    } else {
        motor_.write(kMotorMin);  // Turn motor off
    }
}