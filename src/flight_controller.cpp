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
    noise_cov(seq(0, 2), seq(0, 2)) = MatrixXd::Identity(3, 3) * sq(kPosNoiseStdDev);
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
        VectorXd ang_vel = imu_.ang_velocity();
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

            // Use windspeed data of utility_kf to estimate control surface effect using least squares
            double v_rel = utility_kf_.state_vector()(3);
            roll_ls_.add_row(Vector2d(v_rel * sgn(input_roll_) * sqrt(abs(input_roll_)), 1.), ang_vel(1));
            manage_least_squares(roll_ls_, roll_ls_last_recomp_);
            pitch_ls_.add_row(Vector2d(v_rel * sgn(input_pitch_) * sqrt(abs(input_pitch_)), 1.), ang_vel(0));
            manage_least_squares(pitch_ls_, pitch_ls_last_recomp_);
            yaw_ls_.add_row(Vector2d(v_rel * sgn(input_yaw_) * sqrt(abs(input_yaw_)), 1.), ang_vel(2));
            manage_least_squares(yaw_ls_, yaw_ls_last_recomp_);
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
        target_attitude_ = imu_.attitude();  // Set current attitude as target attitude
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
    update_target_attitude(dt);

    // Generate controls
    double roll_control = 0.;
    double pitch_control = 0.;
    double yaw_control = 0.;
    attitude_controls(roll_control, pitch_control, yaw_control);

    // Save controls in those
    roll = roll_control;
    pitch = pitch_control;
    yaw = yaw_control;
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
    uint8_t satellites = gps_.satellites();
    gps_.time(year, month, day, time);
    sd_.append(String(year) + "-" + String(month) + "-" + String(day) + "-" + String(time) + ":" +
               String(x1, 3) + ";" + String(x2, 3) + ";" + String(x3, 3) + ";" +                    // Position
               String(v1, 3) + ";" + String(v2, 3) + ";" + String(v3, 3) + ";" +                    // Velocity
               String(w, 3) + ";" + String(x, 3) + ";" + String(y, 3) + ";" + String(z, 3) + ";" +  // Attitude
               String(w1, 3) + ";" + String(w2, 3) + ";" + String(w3, 3) + ";" +                    // Wind Speed
               String(cw, 5) + ";" +                                                                // Drag Coefficient
               String(cm, 3) + ";" +                                                                // Motor coefficient
               String(satellites) + "\n");                                                          // Number of active gps satellites
}

void FlightController::manage_least_squares(LeastSquares& ls, int& ls_last_recomp) {
    if (ls.rows() <= kMinLsRows) {
        if (ls.rows() == kMinLsRows) {
            ls.recompute_qr();
            ls_last_recomp = ls.rows();
        }
    } else if (ls.rows() >= kMaxLsRows) {  // Too many new rows
        // -1 shouldn't be necessary but prevents possible off-by-one errors
        ls.remove_n_rows(static_cast<int>(static_cast<double>(kMaxLsRows) * kLsNewRowRatio) - 1);
        ls_last_recomp = ls.rows();
    } else if (1. - static_cast<double>(ls_last_recomp) / static_cast<double>(ls.rows()) > kLsNewRowRatio) {  // Too many new rows
        ls.recompute_qr();
        ls_last_recomp = ls.rows();
    }
}

void FlightController::update_target_attitude(uint32_t dt) {
    double d_sec = static_cast<double>(dt) / 1000000.;

    Matrix3d R = target_attitude_.toRotationMatrix();
    // R = Yaw * Pitch * Roll (First Roll, then Pitch, then finally Yaw)
    Vector3d euler = R.eulerAngles(2, 0, 1);

    // Yaw
    euler(0) += constrain(-input_roll_ * kMaxYawRate + input_yaw_ * kMaxYawRate / 10., -kMaxYawRate, kMaxYawRate) * d_sec;
    // Pitch
    euler(1) = constrain(input_pitch_ * kMaxPitch, -kMaxPitch, kMaxPitch);
    // Roll
    euler(2) = constrain(input_roll_ * kMaxRoll, -kMaxRoll, kMaxRoll);
    // Dependency on Target Heading and speed can be added later to those controls

    // Reassemble Rotation
    R = AngleAxisd(euler(0), Vector3d::UnitZ()) * AngleAxisd(euler(1), Vector3d::UnitX()) * AngleAxisd(euler(2), Vector3d::UnitY());
    target_attitude_ = Quaterniond(R);
}

void FlightController::attitude_controls(double& roll, double& pitch, double& yaw) {
    // The following block doesn't have to be recomputed at every step (Just save K and the three ctrl_0s)

    double v_rel = utility_kf_.state_vector()(3);

    // Add steady state roll, pitch and yaw to compensate for angular drift w0
    Vector2d pitch_ls_sol = pitch_ls_.solve();
    Vector2d roll_ls_sol = roll_ls_.solve();
    Vector2d yaw_ls_sol = yaw_ls_.solve();
    // sqrt(r_0) = -w_0 / (c * v_rel)
    double sqrt_ctrl_0_pitch = -pitch_ls_sol(1) / (pitch_ls_sol(0) * v_rel);
    double sqrt_ctrl_0_roll = -roll_ls_sol(1) / (roll_ls_sol(0) * v_rel);
    double sqrt_ctrl_0_yaw = -yaw_ls_sol(1) / (yaw_ls_sol(0) * v_rel);
    double ctrl_0_pitch = sgn(sqrt_ctrl_0_pitch) * sq(sqrt_ctrl_0_pitch);
    double ctrl_0_roll = sgn(sqrt_ctrl_0_roll) * sq(sqrt_ctrl_0_roll);
    double ctrl_0_yaw = sgn(sqrt_ctrl_0_yaw) * sq(sqrt_ctrl_0_yaw);

    // LQR
    MatrixXd A = MatrixXd::Zero(3, 3);
    MatrixXd B = (Vector3d(pitch_ls_sol(0), roll_ls_sol(0), yaw_ls_sol(0)) * v_rel).asDiagonal();
    MatrixXd Q = Vector3d(100., 100., 100.).asDiagonal();  // LQR Controller tuning in here for now
    MatrixXd R = Vector3d(0.1, 0.1, 0.1).asDiagonal();

    MatrixXd K = lqr(A, B, Q, R);

    // Block that doesn't have to be recomputed every step ends here

    Quaterniond attitude = imu_.attitude();
    Quaterniond d_att = target_attitude_.conjugate() * attitude;
    Vector3d d_theta(2.*d_att.x(), 2.*d_att.y(), 2.*d_att.z());

    Vector3d sqrt_ctrl = -K * d_theta;
    
    pitch = ctrl_0_pitch + sgn(sqrt_ctrl(0)) * sq(sqrt_ctrl(0));
    roll = ctrl_0_roll + sgn(sqrt_ctrl(1)) * sq(sqrt_ctrl(1));
    yaw = ctrl_0_yaw + sgn(sqrt_ctrl(2)) * sq(sqrt_ctrl(2));
}

MatrixXd FlightController::lqr(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R) {
    // Solve the continuous time algebraic Riccati Equation (Like in Wikipedia)
    int n = A.rows();
    MatrixXd R_inv = R.inverse();

    MatrixXd Z = MatrixXd::Zero(2*n, 2*n);
    Z.topLeftCorner(n, n) = A;
    Z.topRightCorner(n, n) = -B * R_inv * B.transpose();
    Z.bottomLeftCorner(n, n) = -Q;
    Z.bottomRightCorner(n, n) = -A.transpose();

    EigenSolver<MatrixXd> eigs(Z);
    VectorXd eig_real = eigs.eigenvalues().real();

    // Find eigenvalues with negative real part. Their corresponding eigenvectors (always real)
    // will span a System from which P can be derived.
    MatrixXd U = MatrixXd::Zero(2*n, n);
    int j = 0;
    for (int i = 0; i < 2*n; ++i) {
        if (eig_real(i) < 0.) {
            U.col(j) = eigs.eigenvectors().col(i);
            ++j;
        }
    }

    MatrixXd U1 = U.topRows(n);
    MatrixXd U2 = U.bottomRows(n);
    MatrixXd P = U2 * U1.inverse();
    MatrixXd K = R_inv * B.transpose() * P;
    return K;
}