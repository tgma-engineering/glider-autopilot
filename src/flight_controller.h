// TODO: Find out gyro noise stddev

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
    static constexpr double kGyroNoiseStdDev = 0.00175;  // 1.75e-3;  // Gyro Output Noise: 0.1 deg/s = 1.75e-3 rad/s
    static constexpr double kGpsXStdDev = 2.5;  // Accuracy of 2.5m CEP stated in datasheet
    static constexpr double kGpsYStdDev = 2.5;
    static constexpr double kGpsZStdDev = 7.5;  // Kind of guessing from observation that vertical measurements could be about 3 times worse
    static constexpr double kVelNoiseStdDev = 0.33;  // Guessing max change is 3m/s in 3s -> 3*stddev = 3m/s / 3s -> stddev of 0.33m/s
    static constexpr double kPosNoiseStdDev = 0.17;  // Guessing max change is 2m in 4s -> stddev of 0.17m
    static constexpr double kWindNoiseStdDev = 0.08;  // Guessing max change is 5m/s in 20s -> stddev of 0.08m/s
    static constexpr double kDragNoiseStdDev = 0.0001;  // This is constant in reality. Just some really small stddev to make sure the kf keeps estimating it
    static constexpr double kMotorNoiseStdDev = 0.0001;  // Same here
    static constexpr double kCtrlSurfParamStdDev = 0.0001;  // Same
    static constexpr double kAngVelDriftStdDev = 0.0001;  // Same for now
    //static constexpr double kDragConstInit = 0.02;  // Drag Acceleration = v^2 * rho*A*Cd/2/m = v^2 * DragConst
    static constexpr double kDragConstInit = 0.14;  // Experimental data
    //static constexpr double kMotorConstInit = 10;  // Imagining that the motor might accelerate with 10m/s^2 if there was no drag
    static constexpr double kMotorConstInit = 1.3;  // Experimental data
    static const String kLogName;
    static const uint32_t kLogTime = 500000;  // In microseconds

    // Flight angle limiters
    static constexpr double kMaxRoll = 30. * PI/180.;  // In Rad
    static constexpr double kMaxPitch = 30. * PI/180.;
    static constexpr double kMaxYawRate = 45. * PI/180.;  // In Rad/s

    // Default values for flight controller, if Kalman Filters and Least Squares estimators are not set up
    static constexpr double kVRelDefault = 15.;  // In m/s
    static constexpr double kW0PitchDefault = 0.;  // In Rad/s
    static constexpr double kW0RollDefault = 0.;  // In Rad/s
    static constexpr double kW0YawDefault = 0.;  // In Rad/s
    static constexpr double kCPitchDefault = 0.9;  // In Rad/m  <- Those are really just wild guesses at this point.
    static constexpr double kCRollDefault = 0.28;  // In Rad/m     Hopefully nobody will every have to rely on them for anything other than testing
    static constexpr double kCYawDefault = 0.03;  // In Rad/m

    static inline double sgn(double a) { return a >= 0. ? 1. : -1.; }
    static inline double sgn_sqrt(double a) { return a >= 0. ? sqrt(a) : -sqrt(-a); }
    static inline double sgn_sq(double a) { return a >= 0. ? sq(a) : -sq(a); }

    FlightController();
    virtual int8_t setup();
    virtual int8_t loop(uint32_t dt);
    void turn_on();
    void set_active();
    void set_inactive();
    void set_input(float roll, float pitch, float yaw, float flap, float motor);
    void controls(float& roll, float& pitch, float& yaw, float& flap, float& motor, uint32_t dt);

private:
    bool is_active_;
    // Controls input to the flight controller (not necessarily those that were applied to the servos)
    float input_roll_;
    float input_pitch_;
    float input_yaw_;
    float input_flap_;
    float input_motor_;
    // Last (applied) output controls are saved here for data analysis purposes
    float last_roll_;
    float last_pitch_;
    float last_yaw_;
    float last_flap_;
    float last_motor_;

    ImuController imu_;
    Quaterniond target_attitude_;
    GpsController gps_;
    bool gps_needs_flush_;

    KalmanFilter position_kf_;   // Estimates position, velocity and accelerometer bias
    KalmanFilter utility_kf_;    // Estimates windspeed, drag constant and motor constant, control surface constants
                                 // Control surface constant equation: v_rel * sqrt(r) * c + w_0 = w, where r is input, w_0 is angular drift and w is angular velocity
    bool is_kf_setup_;
    uint32_t kf_last_propagate_;
    VectorXd utility_kf_last_state_;
    MatrixXd utility_kf_last_cov_;

    SdController sd_;
    uint32_t last_log_elapsed_;  // Time in microseconds since the last Log written to SD card

    // Writes line of format:
    // YYYY-MM-DD-HHMMSSCC:x1;x2;x3;v1;v2;v3;w;x;y;z;w1;w2;w3;cw;cm;crp;crr;cry;w0p;w0r;w0y;sats;dt\n
    void log_state();
    uint32_t ticks_since_log_;

    MatrixXd position_kf_noise_cov() const;
    MatrixXd position_kf_meas_cov() const;
    MatrixXd utility_kf_noise_cov() const;

    void init_utility_kf_(VectorXd pos);

    void update_target_attitude(uint32_t dt);
    void attitude_controls(double& roll, double& pitch, double& yaw);
    MatrixXd lqr(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R);
};

#endif // FLIGHT_CONTROLLER_H_