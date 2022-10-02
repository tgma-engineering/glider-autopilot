#ifndef SBUS_CONTROLLER_H_
#define SBUS_CONTROLLER_H_

#include <Arduino.h>
#include <SBUS2.h>
#include "controller.h"

class SBusController : public Controller {
public:
    static const uint16_t kSBusMin = 0;
    static const uint16_t kSBusMid = 1023;
    static const uint16_t kSBusMax = 2047;
    static const uint8_t kAileronChannel = 0;
    static const uint8_t kElevatorChannel = 1;
    static const uint8_t kRudderChannel = 3;
    static const uint8_t kFlapsChannel = 9;
    static const uint8_t kMotorChannel = 8;
    static const uint8_t kArmSwitchChannel = 13;
    static const uint8_t kManualSwitchChannel = 2;

    SBusController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    
    uint16_t channel(uint8_t ch) const { return channels_[ch]; }
    uint16_t aileron() const { return channels_[kAileronChannel]; }
    uint16_t elevator() const { return channels_[kElevatorChannel]; }
    uint16_t rudder() const { return channels_[kRudderChannel]; }
    uint16_t flaps() const { return channels_[kFlapsChannel]; }
    uint16_t motor() const { return channels_[kMotorChannel]; }
    bool arm_switch() const { return channels_[kArmSwitchChannel] > kSBusMid; }
    bool manual_switch() const { return channels_[kManualSwitchChannel] > kSBusMid; }
    uint8_t fer() const { return fer_; }
    bool failsave() const { return failsave_; }

private:
    uint16_t channels_[18];
    uint8_t fer_;  // Frame Error Rate
    bool failsave_;
};

#endif  // SBUS_CONTROLLER_H_