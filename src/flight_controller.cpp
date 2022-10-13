#include "flight_controller.h"

FlightController::FlightController() : position_kf_(
    [](const VectorXd& x, const VectorXd& u) -> VectorXd {  // State Model: f(x, u)
        // x = x1, x2, x3, v1, v2, v3, a_b1, a_b2, a_b3
        // u = rot.w, rot.x, rot.y, rot.z, a_m1, a_m2, a_m3
        Quaterniond rot(u(0), u(1), u(2), u(3));       // Local-to-Global Quaterion
        VectorXd a_m = u.tail(3);                      // Local frame Acceleration measurement
        VectorXd a_b = x.tail(3);                      // Local frame Accelerometer bias
        VectorXd a = rot._transformVector(a_m - a_b);  // Global frame acceleration with bias correction
        VectorXd x_dot(9);
        x_dot << x(seq(3, 5)),  // x_dot = v
                 a,             // v_dot = R(a_m) - R(a_b)
                 0, 0, 0;       // Accelerometer bias is only driven by noise
        return x_dot;
    },
    [](const VectorXd& x) -> VectorXd {  // Measurement Model: h(x)
        return x.head(3);  // GPS will only measure position
    },
    [](const VectorXd& x, const VectorXd& u) -> MatrixXd {  // State Model Jacobian: df/dx
        Quaterniond rot(u(0), u(1), u(2), u(3));
        MatrixXd rot_mat = rot.toRotationMatrix();
        MatrixXd state_jacobian = MatrixXd::Zero(9, 9);
        state_jacobian(seq(0, 2), seq(3, 5)) = MatrixXd::Identity(3, 3);
        state_jacobian(seq(3, 5), seq(6, 8)) = -rot_mat;
        return state_jacobian;
    },
    [](const VectorXd& x) -> MatrixXd {  // Measurement Model Jacobian: dh/dx
        MatrixXd measure_jacobian = MatrixXd::Zero(3, 9);
        measure_jacobian(all, seq(0, 2)) = MatrixXd::Identity(3, 3);
        return measure_jacobian;
    },
    MatrixXd{  // Continuous Noise Covariance
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, kAccNoiseStdDev*kAccNoiseStdDev, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, kAccNoiseStdDev*kAccNoiseStdDev, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, kAccNoiseStdDev*kAccNoiseStdDev, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, kAccBiasNoiseStdDev*kAccBiasNoiseStdDev, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, kAccBiasNoiseStdDev*kAccBiasNoiseStdDev, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, kAccBiasNoiseStdDev*kAccBiasNoiseStdDev}
    } * static_cast<double>(kImuUpdateFreq),  // Multiplication with update frequency to obtain continuous covariance
    MatrixXd{  // Discrete Measurement Covariance
        {kGpsXStdDev*kGpsXStdDev, 0, 0},
        {0, kGpsYStdDev*kGpsYStdDev, 0},
        {0, 0, kGpsZStdDev*kGpsZStdDev}
    }
) {
    is_active_ = false;
    input_roll_ = 0.f;
    input_pitch_ = 0.f;
    input_yaw_ = 0.f;
    input_flap_ = 0.f;
    input_motor_ = 0.f;
}

int8_t FlightController::setup() {
    if (imu_.setup()) {
        Serial.println("Error: IMU Setup failed");
        return -1;
    }
    if (gps_.setup()) {
        Serial.println("Error: GPS Setup failed");
        return -1;
    }
    return 0;
}

int8_t FlightController::loop(uint32_t dt) {
    // Keep track of attitude and position
    imu_.loop(dt);
    gps_.loop(dt);
    return 0;
}

void FlightController::set_active() {
    if (!is_active_) {
        is_active_ = true;
        target_attitude_ = imu_.attitude();  // Set current attitude as target
    }
}

void FlightController::set_inactive() {
    is_active_ = false;
}

void FlightController::set_input(float roll, float pitch, float yaw, float flap, float motor) {
    input_roll_ = roll;
    input_pitch_ = pitch;
    input_yaw_ = yaw;
    input_flap_ = flap;
    input_motor_ = motor;
}

void FlightController::controls(float& roll, float& pitch, float& yaw, float& flap, float& motor, uint32_t dt) {
    // Update target attitude using the inputs

    // Generate controls

    // Save controls in those
    roll = 0.f;
    pitch = 0.f;
    yaw = 0.f;
    flap = 0.f;
    motor = 0.f;
}