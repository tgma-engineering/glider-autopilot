#ifndef IMU_CONTROLLER_H_
#define IMU_CONTROLLER_H_

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>
#include "controller.h"

using namespace imu;

// TODO: Read out additional data

class ImuController : public Controller {
public:
    static const uint8_t kId = 55;
    static const uint8_t kAddress = 0x28;
    static const uint32_t kSampleDelay = 10000;  // In microseconds

    ImuController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    Quaternion attitude() const { return attitude_; }
    Vector<3> acceleration() const { return acceleration_; }
    Vector<3> ang_velocity() const { return ang_velocity_; }
    Vector<3> gravity() const { return gravity_; }
    
private:
    uint32_t sleep_time_;
    Adafruit_BNO055 bno_;
    Quaternion attitude_;     // Local-to-Global, for (1,0,0,0) y points north, x east and z points up
    Vector<3> acceleration_;  // Acceleration without gravity in body frame
    Vector<3> ang_velocity_;  // Angular Velocity in body frame
    Vector<3> gravity_;       // Gravity in body frame
};

#endif  // IMU_CONTROLLER_H_