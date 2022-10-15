#ifndef FLIGHT_CONTROLLER_H_
#define FLIGHT_CONTROLLER_H_

#include <Arduino.h>
#include <ArduinoEigen.h>
#include "controller.h"
#include "imu_controller.h"
#include "gps_controller.h"
#include "kalman_filter.h"
#include "sd_controller.h"

using namespace Eigen;

class FlightController : public Controller {
public:
    static const uint32_t kImuUpdateFreq = 100;  // In Hz
    static constexpr double kAccNoiseStdDev = 1.;  // Discrete Accelerometer standard deviation
    static constexpr double kAccBiasNoiseStdDev = 1.;
    static constexpr double kGpsXStdDev = 2.5;
    static constexpr double kGpsYStdDev = 2.5;
    static constexpr double kGpsZStdDev = 7.5;
    static const String kLogName;

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

    ImuController imu_;
    Quaterniond target_attitude_;
    GpsController gps_;
    KalmanFilter position_kf_;
    bool is_kf_setup_;
    uint32_t kf_last_propagate_;

    SdController sd_;
};

#endif // FLIGHT_CONTROLLER_H_