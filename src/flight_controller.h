#ifndef FLIGHT_CONTROLLER_H_
#define FLIGHT_CONTROLLER_H_

#include <Arduino.h>
#include "controller.h"

class FlightController : public Controller {
public:
    FlightController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    void set_active();
    void set_inactive();
    void set_input(float roll, float pitch, float yaw, float flap, float motor);
    void controls(float& roll, float& pitch, float& yaw, float& flap, float& motor, uint32_t dt);

private:
    bool is_active_;
    float input_roll_;
    float input_pitch_;
    float input_yaw_;
    float input_flap_;
    float input_motor_;
};

#endif // FLIGHT_CONTROLLER_H_