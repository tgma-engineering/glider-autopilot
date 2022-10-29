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
    [](const VectorXd& x, const VectorXd& u) -> VectorXd {  // Measurement Model: h(x)
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
    [](const VectorXd& x, const VectorXd& u) -> MatrixXd {  // Measurement Model Jacobian: dh/dx
        MatrixXd measure_jacobian = MatrixXd::Zero(3, 9);
        measure_jacobian(all, seq(0, 2)) = MatrixXd::Identity(3, 3);
        return measure_jacobian;
    },
    position_kf_noise_cov(),
    position_kf_meas_cov()
), utility_kf_(
    [](const VectorXd& x, const VectorXd& u) -> VectorXd {  // State Model: f(x, u)
        // x = x1, x2, x3, v_rel, v_wind1, v_wind2, v_wind3, drag_const, motor_const
        // u = rot.w, rot.x, rot.y, rot.z, motor_input
        Quaterniond rot(u(0), u(1), u(2), u(3));  // Local-to-Global Quaterion
        MatrixXd R = rot.toRotationMatrix();
        double v_rel = x(3);
        VectorXd ex = Vector3d(1, 0, 0);
        VectorXd g = Vector3d(0, 0, -9.81);

        VectorXd v = v_rel * (R * ex) + x(seq(4, 6));
        double a_rel = ex.dot(R.transpose() * g) - sq(v_rel) * x(7) + u(4) * x(8);
        VectorXd x_dot(9);
        x_dot << v, a_rel, VectorXd::Zero(5);
        return x_dot;
    },
    [](const VectorXd& x, const VectorXd& u) -> VectorXd {  // Measurement Model: h(x)
        Quaterniond rot(u(0), u(1), u(2), u(3));  // Local-to-Global Quaterion
        MatrixXd R = rot.toRotationMatrix();
        VectorXd ex = Vector3d(1, 0, 0);

        VectorXd v = x(3) * (R * ex) + x(seq(4, 6));
        VectorXd measurement(6);
        measurement << x.head(3), v;
        return measurement;
    },
    [](const VectorXd& x, const VectorXd& u) -> MatrixXd {  // State Model Jacobian: df/dx
        Quaterniond rot(u(0), u(1), u(2), u(3));  // Local-to-Global Quaterion
        MatrixXd R = rot.toRotationMatrix();
        VectorXd ex = Vector3d(1, 0, 0);

        MatrixXd state_jacobian = MatrixXd::Zero(9, 9);
        state_jacobian(seq(0, 2), 3) = R * ex;
        state_jacobian(seq(0, 2), seq(4, 6)) = MatrixXd::Identity(3, 3);
        state_jacobian(3, 3) = -2. * x(3) * x(7);
        state_jacobian(3, 7) = -sq(x(3));
        state_jacobian(3, 8) = u(4);
        return state_jacobian;
    },
    [](const VectorXd& x, const VectorXd& u) -> MatrixXd {  // Measurement Model Jacobian: dh/dx
        Quaterniond rot(u(0), u(1), u(2), u(3));  // Local-to-Global Quaterion
        MatrixXd R = rot.toRotationMatrix();
        VectorXd ex = Vector3d(1, 0, 0);

        MatrixXd measure_jacobian = MatrixXd::Zero(6, 9);
        measure_jacobian(seq(0, 2), seq(0, 2)) = MatrixXd::Identity(3, 3);
        measure_jacobian(seq(3, 5), 3) = R * ex;
        measure_jacobian(seq(3, 5), seq(4, 6)) = MatrixXd::Identity(3, 3);
        return measure_jacobian;
    },
    utility_kf_noise_cov(),
    MatrixXd::Identity(6, 6)  // Dummy. Measure Covariance will be set before every kf update
) {
    is_active_ = false;
    input_roll_ = 0.f;
    input_pitch_ = 0.f;
    input_yaw_ = 0.f;
    input_flap_ = 0.f;
    input_motor_ = 0.f;
    
    last_roll_ = 0.f;
    last_pitch_ = 0.f;
    last_yaw_ = 0.f;
    last_flap_ = 0.f;
    last_motor_ = 0.f;

    is_kf_setup_ = false;
    kf_last_propagate_ = 0;
    last_log_elapsed_ = 0;
}

MatrixXd FlightController::position_kf_noise_cov() const {
    MatrixXd noise_cov = MatrixXd::Zero(9, 9);
    noise_cov(seq(3, 5), seq(3, 5)) = MatrixXd::Identity(3, 3) * sq(kAccNoiseStdDev);
    noise_cov(seq(6, 8), seq(6, 8)) = MatrixXd::Identity(3, 3) * sq(kAccBiasNoiseStdDev);
    // Mulitply by update frequency to turn discrete measurement noise to continuous state noise
    noise_cov *= static_cast<double>(kImuUpdateFreq);
    return noise_cov;
}

MatrixXd FlightController::position_kf_meas_cov() const {
    MatrixXd meas_cov = MatrixXd::Zero(3, 3);  // Discrete Measurement Covariance
    meas_cov(0, 0) = sq(kGpsXStdDev);
    meas_cov(1, 1) = sq(kGpsYStdDev);
    meas_cov(2, 2) = sq(kGpsZStdDev);
    return meas_cov;
}

MatrixXd FlightController::utility_kf_noise_cov() const {
    MatrixXd noise_cov = MatrixXd::Zero(9, 9);
    noise_cov(3, 3) = sq(kVelNoiseStdDev);
    noise_cov(seq(4, 6), seq(4, 6)) = MatrixXd::Identity(3, 3) * sq(kWindNoiseStdDev);
    noise_cov(7, 7) = sq(kDragNoiseStdDev);
    noise_cov(8, 8) = sq(kMotorNoiseStdDev);
    return noise_cov;
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
        // Setup Kalman Filters
        VectorXd pos = gps_.position();
        VectorXd position_x0(9);
        position_x0 << pos, VectorXd::Zero(6);
        MatrixXd position_P0 = VectorXd{{sq(kGpsXStdDev), sq(kGpsYStdDev), sq(kGpsZStdDev),
                                         10., 10., 10.,
                                         10.*kAccBiasNoiseStdDev, 10.*kAccBiasNoiseStdDev, 10.*kAccBiasNoiseStdDev}}.asDiagonal();
        position_kf_.setup(position_x0, position_P0);

        VectorXd utility_x0(9);
        double cw_init = kDragConstInit;
        double cm_init = kMotorConstInit;
        utility_x0 << pos, VectorXd::Zero(4), cw_init, cm_init;
        MatrixXd utility_P0 = VectorXd{{sq(kGpsXStdDev), sq(kGpsYStdDev), sq(kGpsZStdDev),
                                        100.,
                                        25., 25., 25.,
                                        0.0004,
                                        10.}}.asDiagonal();
        utility_kf_.setup(utility_x0, utility_P0);

        is_kf_setup_ = true;
        kf_last_propagate_ = 0;
    }

    if (imu_.new_data_ready()) {
        Quaterniond attitude = imu_.attitude();
        VectorXd acceleration = imu_.acceleration();
        if (is_kf_setup_) {
            VectorXd position_control(7);
            position_control << attitude.w(), attitude.x(), attitude.y(), attitude.z(), acceleration;
            double delta_second = static_cast<double>(kf_last_propagate_) / 1000000.;
            position_kf_.propagate(position_control, delta_second);
            kf_last_propagate_ = 0;

            // Propagate and update utility_kf every imu step. If mcu load is too high, reduce frequency
            double motor_input = last_motor_;
            VectorXd utility_control(5);
            utility_control << attitude.w(), attitude.x(), attitude.y(), attitude.z(), motor_input;
            utility_kf_.propagate(utility_control, delta_second);
            // position_kf is measurement input to utility_kf
            MatrixXd utility_measure_cov = position_kf_.error_cov()(seqN(0, 6), seqN(0, 6));
            utility_kf_.set_measure_cov(utility_measure_cov);
            // Update
            VectorXd utility_measurement = position_kf_.state_vector().head(6);  // x1, x2, x3, v1, v2, v3
            utility_kf_.update(utility_measurement, utility_control);
        }
    }

    if (gps_.new_data_ready()) {
        VectorXd position = gps_.position();
        if (is_kf_setup_) {
            Quaterniond attitude = imu_.attitude();
            VectorXd acceleration = imu_.acceleration();
            VectorXd control(7);
            control << attitude.w(), attitude.x(), attitude.y(), attitude.z(), acceleration;
            position_kf_.update(position, control);
        }
    }

    // Logging stuff
    if (last_log_elapsed_ <= kLogTime) {
        last_log_elapsed_ += dt;
    } else {
        if (is_kf_setup_)  {  // Only log data if valid information is being computed
            last_log_elapsed_ -= kLogTime;
            log_state();
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

    // Last controls are not determined by flight controller so just assume the extern controls
    if (!is_active_) {
        last_roll_ = roll;
        last_pitch_ = pitch;
        last_yaw_ = yaw;
        last_flap_ = flap;
        last_motor_ = motor;
    }
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
    // Save last controls
    last_roll_ = roll;
    last_pitch_ = pitch;
    last_yaw_ = yaw;
    last_flap_ = flap;
    last_motor_ = motor;
}

void FlightController::log_state() const {
    // Write log entry
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint32_t time;
    VectorXd position_state = position_kf_.state_vector();
    double x1 = position_state(0);
    double x2 = position_state(1);
    double x3 = position_state(2);
    double v1 = position_state(3);
    double v2 = position_state(4);
    double v3 = position_state(5);
    Quaterniond attitude = imu_.attitude();
    double w = attitude.w();
    double x = attitude.x();
    double y = attitude.y();
    double z = attitude.z();
    VectorXd utility_state = utility_kf_.state_vector();
    double w1 = utility_state(4);  // Windspeed
    double w2 = utility_state(5);
    double w3 = utility_state(6);
    double cw = utility_state(7);  // Drag coefficient
    double cm = utility_state(8);  // Motor coefficient
    gps_.time(year, month, day, time);
    sd_.append(String(year) + "-" + String(month) + "-" + String(day) + "-" + String(time) + ":" +
                String(x1, 3) + ";" + String(x2, 3) + ";" + String(x3, 3) + ";" +                    // Position
                String(v1, 3) + ";" + String(v2, 3) + ";" + String(v3, 3) + ";" +                    // Velocity
                String(w, 3) + ";" + String(x, 3) + ";" + String(y, 3) + ";" + String(z, 3) + ";" +  // Attitude
                String(w1, 3) + ";" + String(w2, 3) + ";" + String(w3, 3) + ";" +                    // Wind Speed
                String(cw, 5) + ";" +                                                                // Drag Coefficient
                String(cm, 3) + "\n");                                                               // Motor coefficient
}