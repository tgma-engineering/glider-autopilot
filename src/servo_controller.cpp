#include "servo_controller.h"

const float ServoController::kFlapAileronLCross =
    (float)(ServoController::kAileronLMax-ServoController::kAileronLMid) /
    (float)(ServoController::kFlapMid-ServoController::kFlapMin);
const float ServoController::kFlapAileronRCross =
    -(float)(ServoController::kAileronRMid-ServoController::kAileronRMin) /
    (float)(ServoController::kFlapMid-ServoController::kFlapMin);

uint16_t ServoController::sbus_to_time(uint16_t sbus_channel, uint16_t pulse_min, uint16_t pulse_mid, uint16_t pulse_max, bool invert) {
    uint16_t channel = sbus_channel;
    if (invert)
        channel = kSBusMax - sbus_channel;
    
    if (channel > kSBusMid) {
        return map(channel, kSBusMid, kSBusEffMax, pulse_mid, pulse_max);
    } else {
        return map(channel, kSBusEffMin, kSBusMid, pulse_min, pulse_mid);
    }
}

// Computes if motor speed is above or below midpoint given the SBus input
bool ServoController::is_motor_on(uint16_t sbus_motor) {
    uint16_t motor_time = constrain(sbus_to_time(sbus_motor, kMotorMin, kMotorMid, kMotorMax, kMotorSBusInv), kMotorMin, kMotorMax);
    if (motor_time > 1472)
        return true;
    return false;
}

ServoController::ServoController() {
    ready_timer = 0;
}

int8_t ServoController::setup() {
    bool error = false;
    if (!aileron_l_.attach(kAileronLGpio)) {
        error = true;
        Serial.println("Error: Left Aileron Servo Setup failed");
    }
    if (!aileron_r_.attach(kAileronRGpio)) {
        error = true;
        Serial.println("Error: Right Aileron Servo Setup failed");
    }
    if (!elevator_.attach(kElevatorGpio)) {
        error = true;
        Serial.println("Error: Elevator Servo Setup failed");
    }
    if (!rudder_.attach(kRudderGpio)) {
        error = true;
        Serial.println("Error: Rudder Servo Setup failed");
    }
    if (!flap_l_.attach(kFlapLGpio)) {
        error = true;
        Serial.println("Error: Left Flap Servo Setup failed");
    }
    if (!flap_r_.attach(kFlapRGpio)) {
        error = true;
        Serial.println("Error: Right Flap failed");
    }
    if (!motor_.attach(kMotorGpio)) {
        error = true;
        Serial.println("Error: Motor Servo Setup failed");
    }
    
    if (error)
        return -1;
    return 0;
}

int8_t ServoController::loop(uint32_t dt) {
    if (ready_timer <= kReadyTime)
        ready_timer += dt;
    return 0;
}

bool ServoController::is_ready_reset() {
    if (ready_timer < kReadyTime) {
        return false;
    } else {
        ready_timer = 0;
        return true;
    }
}

void ServoController::set_from_sbus(const SBusController& sbus, bool write_motor) {
    if (!is_ready_reset())
        return;  // Only set servos as often as necessary

    uint16_t flap_sbus;
    if (kFlapSBusInv) {
        flap_sbus = kSBusMax - sbus.flap();
    } else {
        flap_sbus = sbus.flap();
    }
    uint16_t flap_time = constrain(map(flap_sbus, kSBusEffMin, kSBusEffMax - kFlapSBusDeadZone, kFlapMin, kFlapMid), kFlapMin, kFlapMid);
    Serial.print(flap_sbus);
    Serial.print(" ");
    Serial.println(flap_time);
    flap_l_.write(flap_time);
    flap_r_.write(flap_time);

    uint16_t aileron_l_flapping = kAileronLMid + static_cast<int16_t>(static_cast<float>(kFlapMid-flap_time)*kFlapAileronLCross);
    uint16_t aileron_r_flapping = kAileronRMid + static_cast<int16_t>(static_cast<float>(kFlapMid-flap_time)*kFlapAileronRCross);
    uint16_t aileron_l_time = constrain(sbus_to_time(sbus.aileron(), kAileronLMin, aileron_l_flapping, kAileronLMax, kAileronLSBusInv), kAileronLMin, kAileronLMax);
    uint16_t aileron_r_time = constrain(sbus_to_time(sbus.aileron(), kAileronRMin, aileron_r_flapping, kAileronRMax, kAileronRSBusInv), kAileronRMin, kAileronRMax);
    aileron_l_.write(constrain(aileron_l_time, kAileronLMin, kAileronLMax));
    aileron_r_.write(constrain(aileron_r_time, kAileronRMin, kAileronRMax));

    elevator_.write(constrain(sbus_to_time(sbus.elevator(), kElevatorMin, kElevatorMid, kElevatorMax, kElevatorSBusInv), kElevatorMin, kElevatorMax));
    rudder_.write(constrain(sbus_to_time(sbus.rudder(), kRudderMin, kRudderMid, kRudderMax, kRudderSBusInv), kRudderMin, kRudderMax));
    
    if (write_motor) {
        motor_.write(constrain(sbus_to_time(sbus.motor(), kMotorMin, kMotorMid, kMotorMax, kMotorSBusInv), kMotorMin, kMotorMax));
    } else {
        motor_.write(kMotorMin);  // Turn motor off
    }
}