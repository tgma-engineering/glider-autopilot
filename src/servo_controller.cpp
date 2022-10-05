#include "servo_controller.h"

const float ServoController::kFlapAileronLCross =
    (float)(ServoController::kAileronLMax-ServoController::kAileronLMid) /
    (float)(ServoController::kFlapMid-ServoController::kFlapMin);
const float ServoController::kFlapAileronRCross =
    -(float)(ServoController::kAileronRMid-ServoController::kAileronRMin) /
    (float)(ServoController::kFlapMid-ServoController::kFlapMin);

long ServoController::double_map(long a, long a_min, long a_mid, long a_max, long b_min, long b_mid, long b_max) {
    if (a >= a_mid) {
        return map(a, a_mid, a_max, b_mid, b_max);
    } else {
        return map(a, a_min, a_mid, b_min, b_mid);
    }
}

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
    if (motor_time > kMotorMid)
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
    flap_l_.write(flap_time);
    flap_r_.write(flap_time);

    uint16_t aileron_l_flapping = kAileronLMid + static_cast<int16_t>(static_cast<float>(kFlapMid-flap_time)*kFlapAileronLCross);
    uint16_t aileron_r_flapping = kAileronRMid + static_cast<int16_t>(static_cast<float>(kFlapMid-flap_time)*kFlapAileronRCross);
    uint16_t aileron_l_time = constrain(sbus_to_time(sbus.aileron(), kAileronLMin, aileron_l_flapping, kAileronLMax, kAileronLSBusInv), kAileronLMin, kAileronLMax);
    uint16_t aileron_r_time = constrain(sbus_to_time(sbus.aileron(), kAileronRMin, aileron_r_flapping, kAileronRMax, kAileronRSBusInv), kAileronRMin, kAileronRMax);
    aileron_l_.write(aileron_l_time);
    aileron_r_.write(aileron_r_time);

    elevator_.write(constrain(sbus_to_time(sbus.elevator(), kElevatorMin, kElevatorMid, kElevatorMax, kElevatorSBusInv), kElevatorMin, kElevatorMax));
    rudder_.write(constrain(sbus_to_time(sbus.rudder(), kRudderMin, kRudderMid, kRudderMax, kRudderSBusInv), kRudderMin, kRudderMax));
    
    if (write_motor) {
        motor_.write(constrain(sbus_to_time(sbus.motor(), kMotorMin, kMotorMid, kMotorMax, kMotorSBusInv), kMotorMin, kMotorMax));
    } else {
        motor_.write(kMotorMin);  // Turn motor off
    }
}

// Roll, Yaw: -1 - right; 0 - mid; 1 - left
// Pitch: -1 - down; 0 - mid; 1 - up
// Flap: 0 - inactive; 1 - active
// Motor: 0 - off; 1 - on
void ServoController::set(float roll, float pitch, float yaw, float flap, float motor) {
    static const int16_t kRangeL = 1000;
    static const float kRangeF = 1000.f;

    if (!is_ready_reset())
        return;  // Only set servos as often as necessary
    
    int16_t aileron_i = static_cast<int16_t>(roll * kRangeF);
    int16_t elevator_i = static_cast<int16_t>(pitch * kRangeF);
    int16_t rudder_i = static_cast<int16_t>(yaw * kRangeF);
    int16_t flap_i = static_cast<int16_t>(flap * kRangeF);
    int16_t motor_i = static_cast<int16_t>(motor * kRangeF);

    uint16_t flap_time = constrain(map(flap_i, 0, kRangeL, kFlapMid, kFlapMin), kFlapMin, kFlapMid);
    flap_l_.write(flap_time);
    flap_r_.write(flap_time);

    uint16_t aileron_l_flapping = kAileronLMid + static_cast<int16_t>(static_cast<float>(kFlapMid-flap_time)*kFlapAileronLCross);
    uint16_t aileron_r_flapping = kAileronRMid + static_cast<int16_t>(static_cast<float>(kFlapMid-flap_time)*kFlapAileronRCross);
    uint16_t aileron_l_time = constrain(double_map(aileron_i, -kRangeL, 0, kRangeL, kAileronLMin, aileron_l_flapping, kAileronLMax), kAileronLMin, kAileronLMax);
    uint16_t aileron_r_time = constrain(double_map(aileron_i, -kRangeL, 0, kRangeL, kAileronRMin, aileron_r_flapping, kAileronRMax), kAileronRMin, kAileronRMax);
    aileron_l_.write(aileron_l_time);
    aileron_r_.write(aileron_r_time);

    elevator_.write(constrain(double_map(elevator_i, -kRangeL, 0, kRangeL, kElevatorMin, kElevatorMid, kElevatorMax), kElevatorMin, kElevatorMax));
    rudder_.write(constrain(double_map(rudder_i, -kRangeL, 0, kRangeL, kRudderMin, kRudderMid, kRudderMax), kRudderMin, kRudderMax));
    
    motor_.write(constrain(map(motor_i, 0, kRangeL, kMotorMin, kMotorMax), kMotorMin, kMotorMax));
}