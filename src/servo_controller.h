#ifndef SERVO_CONTROLLER_H_
#define SERVO_CONTROLLER_H_

#include <Arduino.h>
#include <ESP32Servo.h>
#include "controller.h"
#include "sbus_controller.h"

class ServoController : public Controller {
public:
    static const uint16_t kPulsMin = 544;  // In microseconds
    static const uint16_t kPulsMax = 2400;
    static const uint16_t kSBusMin = 0;
    static const uint16_t kSBusMax = 2047;

    static const uint8_t kAileronLGpio = 14;
    static const uint8_t kAileronRGpio = 27;
    static const uint8_t kElevatorGpio = 15;
    static const uint8_t kRudderGpio = 2;
    static const uint8_t kFlapLGpio = 13;
    static const uint8_t kFlapRGpio = 12;
    static const uint8_t kMotorGpio = 4;

    // Pulses in microseconds
    static const uint16_t kAileronMin = 1090;  // Right
    static const uint16_t kAileronMid = 1470;
    static const uint16_t kAileronMax = 1850;  // Left
    static const bool kAileronSBusInv = true;

    static const uint16_t kElevatorMin = 863;   // Down
    static const uint16_t kElevatorMid = 1499;
    static const uint16_t kElevatorMax = 2272;  // Up
    static const bool kElevatorSBusInv = false;

    static const uint16_t kRudderMin = 862;   // Right
    static const uint16_t kRudderMid = 1470;
    static const uint16_t kRudderMax = 2080;  // Left
    static const bool kRudderSBusInv = true;

    static const uint16_t kFlapMin = 1200;  // Active
    static const uint16_t kFlapMid = 2080;  // Inactive
    static const uint16_t kFlapMax = 2100;  // Not actually in use
    static const bool kFlapSBusInv = true;

    static const uint16_t kMotorMin = 863;  // Off
    static const uint16_t kMotorMid = 1472;  // There is no real motor midpoint
    static const uint16_t kMotorMax = 2082;  // On
    static const bool kMotorSBusInv = true;

    static uint16_t sbus_to_time(uint16_t sbus_channel, bool invert = false);
    static bool is_motor_on(uint16_t sbus_motor);

    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);  // Not in use right now

    void set_from_sbus(const SBusController& sbus, bool write_motor = true);

private:
    Servo aileron_l_;
    Servo aileron_r_;
    Servo elevator_;
    Servo rudder_;
    Servo flap_l_;
    Servo flap_r_;
    Servo motor_;
};

#endif  // SERVO_CONTROLLER_H_