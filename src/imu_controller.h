#ifndef IMU_CONTROLLER_H_
#define IMU_CONTROLLER_H_

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <ArduinoEigen.h>
#include "controller.h"

using namespace Eigen;

class ImuController : public Controller {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Makes fixed-size Eigen Objects work with new operator

    static const uint8_t kId = 55;
    static const uint8_t kAddress = 0x28;
    static const uint32_t kSampleDelay = 10000;  // In microseconds

    ImuController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    Quaterniond attitude() const { return attitude_; }
    Vector3d acceleration() const { return acceleration_; }
    Vector3d ang_velocity() const { return ang_velocity_; }
    Vector3d gravity() const { return gravity_; }
    bool new_data_ready();
    
private:
    uint32_t sleep_time_;
    Adafruit_BNO055 bno_;
    Quaterniond attitude_;   // Local-to-Global, for (1,0,0,0) y points north, x east and z points up
    Vector3d acceleration_;  // Acceleration without gravity in body frame
    Vector3d ang_velocity_;  // Angular Velocity in body frame
    Vector3d gravity_;       // Gravity in body frame
    bool new_data_ready_;
};

#endif  // IMU_CONTROLLER_H_