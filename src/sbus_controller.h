#ifndef SBUS_CONTROLLER_H_
#define SBUS_CONTROLLER_H_

#include <Arduino.h>
#include <SBUS2.h>
#include "controller.h"

class SBusController : public Controller {
public:
    static const uint16_t kSBusMin = 0;
    static const uint16_t kSBusEffMin = 350;
    static const uint16_t kSBusThird = 682;
    static const uint16_t kSBusMid = 1023;
    static const uint16_t kSBusEffMax = 1697;
    static const uint16_t kSBusMax = 2047;

    static const uint8_t kAileronChannel = 0;
    static const uint8_t kElevatorChannel = 1;
    static const uint8_t kRudderChannel = 3;
    static const uint8_t kFlapChannel = 9;
    static const uint8_t kMotorChannel = 8;
    static const uint8_t kArmSwitchChannel = 13;  // The one left of SB. Forward is disarmed, backward is armed
    static const uint8_t kManualSwitchChannel = 2;  // SG, Down is manual

    // These two functions don't access member variables, they are intended as utilities only
    static bool is_armed(uint16_t sbus_arm_switch);
    static uint8_t manual_switch_state(uint16_t sbus_manual_switch);

    SBusController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    
    uint16_t channel(uint8_t c) const { return channels_[c]; }
    uint16_t aileron() const { return channels_[kAileronChannel]; }
    uint16_t elevator() const { return channels_[kElevatorChannel]; }
    uint16_t rudder() const { return channels_[kRudderChannel]; }
    uint16_t flap() const { return channels_[kFlapChannel]; }
    uint16_t motor() const { return channels_[kMotorChannel]; }
    uint16_t arm_switch() const { return channels_[kArmSwitchChannel]; }
    uint16_t manual_switch() const { return channels_[kManualSwitchChannel]; }
    uint8_t fer() const { return fer_; }
    bool failsave() const { return failsave_; }

    void get_controls(float& roll_out, float& pitch_out, float& yaw_out, float& flap_out, float& motor_out);

private:
    uint16_t channels_[18];
    uint8_t fer_;  // Frame Error Rate
    bool failsave_;
};

#endif  // SBUS_CONTROLLER_H_