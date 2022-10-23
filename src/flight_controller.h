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
    // Accelerometer default bandwidth: 62.5 Hz
    // Acc-Noise: 150 ug/sqrt(Hz)
    // => Acc-Noise-Std-Dev = 150e-6 g/sqrt(Hz) * 9.81ms^-2/g * sqrt(62.5 Hz) = 0.01163 m/s^2
    static constexpr double kAccNoiseStdDev = 0.01163;  // Discrete Accelerometer standard deviation in m/s^2
    //static constexpr double kAccNoiseStdDev = 0.023;  // Noise seems to be a little higher in reality due to uncertainty in gravity vector
    static constexpr double kAccBiasNoiseStdDev = 0.01163;  // Guessing this could be in the same range as the Acc-Noise
    static constexpr double kGpsXStdDev = 2.5;  // Accuracy of 2.5m CEP stated in datasheet
    static constexpr double kGpsYStdDev = 2.5;
    static constexpr double kGpsZStdDev = 7.5;  // Kind of guessing from observation that vertical measurements could be about 3 times worse
    static const String kLogName;
    static const uint32_t kLogTime = 50000;  // In microseconds

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
    uint32_t last_log_elapsed_;  // Time in microseconds since the last Log written to SD card

    // Writes line of format:
    // YYYY-MM-DD-HHMMSSCC:x1;x2;x3;v1;v2;v3;w;x;y;z\n
    void log_state() const;

    MatrixXd position_kf_noise_cov() const;
    MatrixXd position_kf_meas_cov() const;
};

#endif // FLIGHT_CONTROLLER_H_