#include "flight_controller.h"

const String FlightController::kLogName = "autopilot.log";

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
        VectorXd measurement = x.head(3);  // GPS will only measure position
        return measurement;
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
    position_kf_noise_cov(),
    position_kf_meas_cov()
) {
    is_active_ = false;
    input_roll_ = 0.f;
    input_pitch_ = 0.f;
    input_yaw_ = 0.f;
    input_flap_ = 0.f;
    input_motor_ = 0.f;
    is_kf_setup_ = false;
    kf_last_propagate_ = 0;
    last_log_elapsed_ = 0;
}

MatrixXd FlightController::position_kf_noise_cov() const {
    MatrixXd noise_cov{  // Continuous Noise Covariance
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, kAccNoiseStdDev*kAccNoiseStdDev, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, kAccNoiseStdDev*kAccNoiseStdDev, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, kAccNoiseStdDev*kAccNoiseStdDev, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, kAccBiasNoiseStdDev*kAccBiasNoiseStdDev, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, kAccBiasNoiseStdDev*kAccBiasNoiseStdDev, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, kAccBiasNoiseStdDev*kAccBiasNoiseStdDev}
    };  // Multiplication with update frequency to obtain continuous covariance
    noise_cov *= static_cast<double>(kImuUpdateFreq);
    return noise_cov;
}

MatrixXd FlightController::position_kf_meas_cov() const {
    MatrixXd meas_cov{  // Discrete Measurement Covariance
        {kGpsXStdDev*kGpsXStdDev, 0, 0},
        {0, kGpsYStdDev*kGpsYStdDev, 0},
        {0, 0, kGpsZStdDev*kGpsZStdDev}
    };
    return meas_cov;
}

int8_t FlightController::setup() {
    Serial.println("Setup IMU ...");
    if (imu_.setup()) {
        Serial.println("Error: IMU Setup failed");
        return -1;
    }
    Serial.println("Setup GPS ...");
    if (gps_.setup()) {
        Serial.println("Error: GPS Setup failed");
        return -1;
    }
    Serial.println("Setup SD ...");
    if (sd_.setup(kLogName)) {
        Serial.println("Error: SD Setup failed");
        return -1;
    }
    return 0;
}

int8_t FlightController::loop(uint32_t dt) {
    // Keep track of attitude and position
    imu_.loop(dt);
    gps_.loop(dt);

    kf_last_propagate_ += dt;

    if (gps_.is_valid() && !is_kf_setup_) {
        // Setup Kalman Filter
        VectorXd pos = gps_.position();
        VectorXd x0(9);
        x0 << pos, VectorXd::Zero(6);
        MatrixXd P0 = VectorXd{{kGpsXStdDev*kGpsXStdDev, kGpsYStdDev*kGpsYStdDev, kGpsZStdDev*kGpsZStdDev,
                                10., 10., 10.,
                                10.*kAccBiasNoiseStdDev, 10.*kAccBiasNoiseStdDev, 10.*kAccBiasNoiseStdDev}}.asDiagonal();
        position_kf_.setup(x0, P0);
        is_kf_setup_ = true;
        kf_last_propagate_ = 0;
    }

    if (imu_.new_data_ready()) {
        Quaterniond attitude = imu_.attitude();
        VectorXd acceleration = imu_.acceleration();
        if (is_kf_setup_) {
            VectorXd control(7);
            control << attitude.w(), attitude.x(), attitude.y(), attitude.z(), acceleration;
            double delta_second = static_cast<double>(kf_last_propagate_) / 1000000.;
            position_kf_.propagate(control, delta_second);
            kf_last_propagate_ = 0;
        }
    }

    if (gps_.new_data_ready()) {
        VectorXd position = gps_.position();
        if (is_kf_setup_) {
            position_kf_.update(position);
        }
    }

    // Logging stuff
    if (last_log_elapsed_ <= kLogTime) {
        last_log_elapsed_ += dt;
    } else {
        if (is_kf_setup_)  {  // Only log data if valid information is being computed
            last_log_elapsed_ -= kLogTime;
            log_state();
            
            VectorXd pos = position_kf_.state_vector()(seq(0, 2));
            Serial.print(pos(0));
            Serial.print("\t");
            Serial.print(pos(1));
            Serial.print("\t");
            Serial.print(pos(2));
            Serial.print("\t");
            Serial.println(gps_.satellites());
        }
    }

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
    roll = input_roll_;
    pitch = input_pitch_;
    yaw = input_yaw_;
    flap = input_flap_;
    motor = input_motor_;
}

void FlightController::log_state() const {
    // Write log entry
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint32_t time;
    VectorXd state = position_kf_.state_vector();
    double x1 = state(0);
    double x2 = state(1);
    double x3 = state(2);
    double v1 = state(3);
    double v2 = state(4);
    double v3 = state(5);
    Quaterniond attitude = imu_.attitude();
    double w = attitude.w();
    double x = attitude.x();
    double y = attitude.y();
    double z = attitude.z();
    gps_.time(year, month, day, time);
    sd_.append(String(year) + "-" + String(month) + "-" + String(day) + "-" + String(time) + ":" +
                String(x1, 3) + ";" + String(x2, 3) + ";" + String(x3, 3) + ";" +
                String(v1, 3) + ";" + String(v2, 3) + ";" + String(v3, 3) + ";" +
                String(w, 3) + ";" + String(x, 3) + ";" + String(y, 3) + ";" + String(z, 3) + "\n");
}