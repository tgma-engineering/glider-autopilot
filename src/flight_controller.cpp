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
        // x = x1, x2, x3, v_rel, v_wind1, v_wind2, v_wind3, drag_const, motor_const, c_pitch, c_roll, c_yaw, w0_pitch, w0_roll, w0_yaw
        // u = rot.w, rot.x, rot.y, rot.z, motor_input, pitch_input, roll_input, yaw_input
        Quaterniond rot(u(0), u(1), u(2), u(3));  // Local-to-Global Quaterion
        MatrixXd R = rot.toRotationMatrix();
        double v_rel = x(3);
        VectorXd ex = Vector3d(1, 0, 0);
        VectorXd g = Vector3d(0, 0, -9.81);

        VectorXd v = v_rel * (R * ex) + x(seq(4, 6));
        double a_rel = ex.dot(R.transpose() * g) - sq(v_rel) * x(7) + u(4) * x(8);
        VectorXd x_dot(15);
        x_dot << v, a_rel, VectorXd::Zero(11);
        return x_dot;
    },
    [](const VectorXd& x, const VectorXd& u) -> VectorXd {  // Measurement Model: h(x, u)
        // h = x1, x2, x3, v1, v2, v3, ang_vel_pitch, ang_vel_roll, ang_vel_yaw
        Quaterniond rot(u(0), u(1), u(2), u(3));  // Local-to-Global Quaterion
        MatrixXd R = rot.toRotationMatrix();
        VectorXd ex = Vector3d(1, 0, 0);

        VectorXd v = x(3) * (R * ex) + x(seq(4, 6));
        // w = v_rel * sqrt(input) * c + w0
        VectorXd w = x(3) * u(seq(5, 7)).unaryExpr(function<double(double)>(sgn_sqrt)).cwiseProduct(x(seq(9, 11))) + x(seq(12, 14));
        VectorXd measurement(9);
        measurement << x.head(3), v, w;
        return measurement;
    },
    [](const VectorXd& x, const VectorXd& u) -> MatrixXd {  // State Model Jacobian: df/dx
        Quaterniond rot(u(0), u(1), u(2), u(3));  // Local-to-Global Quaterion
        MatrixXd R = rot.toRotationMatrix();
        VectorXd ex = Vector3d(1, 0, 0);

        MatrixXd state_jacobian = MatrixXd::Zero(15, 15);
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

        MatrixXd measure_jacobian = MatrixXd::Zero(9, 15);
        measure_jacobian(seq(0, 2), seq(0, 2)) = MatrixXd::Identity(3, 3);
        measure_jacobian(seq(3, 5), 3) = R * ex;
        measure_jacobian(seq(3, 5), seq(4, 6)) = MatrixXd::Identity(3, 3);
        measure_jacobian(seq(6, 8), 3) = u(seq(5, 7)).unaryExpr(function<double(double)>(sgn_sqrt)).cwiseProduct(x(seq(9, 11)));
        measure_jacobian(seq(6, 8), seq(9, 11)) = x(3) * u(seq(5, 7)).unaryExpr(function<double(double)>(sgn_sqrt)).asDiagonal();
        measure_jacobian(seq(6, 8), seq(12, 14)) = MatrixXd::Identity(3, 3);
        return measure_jacobian;
    },
    utility_kf_noise_cov(),
    MatrixXd::Identity(9, 9)  // Dummy. Measure Covariance will be set before every kf update
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

    ticks_since_log_ = 0;

    is_kf_setup_ = false;
    kf_last_propagate_ = 0;
    last_log_elapsed_ = 0;

    utility_kf_last_state_ = VectorXd::Zero(1);
    utility_kf_last_cov_ = MatrixXd::Zero(1, 1);
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
    MatrixXd noise_cov = MatrixXd::Zero(15, 15);
    noise_cov(seq(0, 2), seq(0, 2)) = MatrixXd::Identity(3, 3) * sq(kPosNoiseStdDev);
    noise_cov(3, 3) = sq(kVelNoiseStdDev);
    noise_cov(seq(4, 6), seq(4, 6)) = MatrixXd::Identity(3, 3) * sq(kWindNoiseStdDev);
    noise_cov(7, 7) = sq(kDragNoiseStdDev);
    noise_cov(8, 8) = sq(kMotorNoiseStdDev);
    noise_cov(seq(9, 11), seq(9, 11)) = MatrixXd::Identity(3, 3) * sq(kCtrlSurfParamStdDev);
    noise_cov(seq(12, 14), seq(12, 14)) = MatrixXd::Identity(3, 3) * sq(kAngVelDriftStdDev);
    return noise_cov;
}

void FlightController::init_utility_kf_(VectorXd pos) {
    VectorXd utility_x0(15);
    double dragConstInit = kDragConstInit;  // Eigen doesn't work with constants directly apparently
    double motorConstInit = kMotorConstInit;
    double cPitchDefault = kCPitchDefault;
    double cRollDefault = kCRollDefault;
    double cYawDefault = kCYawDefault;
    double w0PitchDefault = kW0PitchDefault;
    double w0RollDefault = kW0RollDefault;
    double w0YawDefault = kW0YawDefault;
    utility_x0 << pos, VectorXd::Zero(4), dragConstInit, motorConstInit, cPitchDefault, cRollDefault, cYawDefault, w0PitchDefault, w0RollDefault, w0YawDefault;
    MatrixXd utility_P0 = VectorXd{{sq(kGpsXStdDev), sq(kGpsYStdDev), sq(kGpsZStdDev),
                                    100.,
                                    25., 25., 25.,
                                    0.0004,
                                    10.,
                                    sq(kCPitchDefault/3.), sq(kCRollDefault/3.), sq(kCYawDefault/3.),
                                    sq(kW0PitchDefault/3.), sq(kW0RollDefault/3.), sq(kW0YawDefault/3.)}}.asDiagonal();
    utility_kf_.setup(utility_x0, utility_P0);

    utility_kf_last_state_ = utility_x0;
    utility_kf_last_cov_ = utility_P0;
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

        init_utility_kf_(pos);
        
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
            VectorXd utility_control(8);
            utility_control << attitude.w(), attitude.x(), attitude.y(), attitude.z(), last_motor_, last_pitch_, last_roll_, last_yaw_;
            utility_kf_.propagate(utility_control, delta_second);
            // position_kf is measurement input to utility_kf
            MatrixXd utility_measure_cov(9, 9);
            utility_measure_cov(seq(0, 5), seq(0, 5)) = position_kf_.error_cov()(seq(0, 5), seq(0, 5));
            utility_measure_cov(seq(6, 8), seq(6, 8)) = MatrixXd::Identity(3, 3) * sq(kGyroNoiseStdDev);
            utility_kf_.set_measure_cov(utility_measure_cov);
            // Update
            VectorXd utility_measurement(9);
            utility_measurement(seq(0, 5)) = position_kf_.state_vector().head(6);  // x1, x2, x3, v1, v2, v3
            utility_measurement(seq(6, 8)) = ang_vel;
            utility_kf_.update(utility_measurement, utility_control);
            // If something goes wrong (nan matrices):
            static uint8_t utility_kf_fail_cnt = 0;
            if (utility_kf_.state_vector().array().isNaN().sum()) {
                // Reset to last valid
                utility_kf_.setup(utility_kf_last_state_, utility_kf_last_cov_);
                ++utility_kf_fail_cnt;
            } else {
                utility_kf_last_state_ = utility_kf_.state_vector();
                utility_kf_last_cov_ = utility_kf_.error_cov();
                utility_kf_fail_cnt = 0;
            }
            // If there were more than 10 fails in a row, reset utility_kf back to default values
            if (utility_kf_fail_cnt > 5) {
                init_utility_kf_(position_kf_.state_vector()(seq(0, 2)));
                utility_kf_fail_cnt = 0;
            }
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
    if (is_kf_setup_) {  // Only log data if valid information is being computed
        ++ticks_since_log_;
        if (last_log_elapsed_ <= kLogTime) {
            last_log_elapsed_ += dt;
        } else {
            last_log_elapsed_ -= kLogTime;
            log_state();
            ticks_since_log_ = 0;
        }
    } else {
        ticks_since_log_ = 0;
    }
    
    return 0;
}

void FlightController::turn_on() {
    gps_.flush_serial();  // Remove all the old data from GPS Serial
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

void FlightController::log_state() {
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
    VectorXd ctrl_surf_params = utility_state(seq(9, 11));
    VectorXd ang_vel_drift_params = utility_state(seq(12, 14));
    uint8_t satellites = gps_.satellites();
    double avg_tick_micros = kLogTime / ticks_since_log_;
    gps_.time(year, month, day, time);
    sd_.append(String(year) + "-" + String(month) + "-" + String(day) + "-" + String(time) + ":" +
               String(x1, 3) + ";" + String(x2, 3) + ";" + String(x3, 3) + ";" +                                                                 // Position
               String(v1, 3) + ";" + String(v2, 3) + ";" + String(v3, 3) + ";" +                                                                 // Velocity
               String(w, 3) + ";" + String(x, 3) + ";" + String(y, 3) + ";" + String(z, 3) + ";" +                                               // Attitude
               String(w1, 3) + ";" + String(w2, 3) + ";" + String(w3, 3) + ";" +                                                                 // Wind Speed
               String(cw, 5) + ";" +                                                                                                             // Drag Coefficient
               String(cm, 3) + ";" +                                                                                                             // Motor coefficient
               String(ctrl_surf_params(0), 3) + ";" + String(ctrl_surf_params(1), 3) + ";" + String(ctrl_surf_params(2), 3) + ";" +              // Control surface coefficients
               String(ang_vel_drift_params(0), 5) + ";" + String(ang_vel_drift_params(1), 5) + ";" + String(ang_vel_drift_params(2), 5) + ";" +  // Angular drift
               String(satellites) + ";" +                                                                                                        // Number of active gps satellites
               String(avg_tick_micros) + "\n");                                                                                                  // Average duration of tick in microseconds
}

void FlightController::update_target_attitude(uint32_t dt) {
    double d_sec = static_cast<double>(dt) / 1000000.;

    Matrix3d R = target_attitude_.toRotationMatrix();
    // R = Yaw * Pitch * Roll (First Roll, then Pitch, then finally Yaw)
    Vector3d euler = R.eulerAngles(2, 0, 1);
    // eulerAngles is bullshit because it restricts the first angle (yaw in this case) to
    // a range of 0 to pi. I need the second angle (pitch) to be restricted to -pi/2 to pi/2 though.
    // This means that every time yaw would become negative the other angles just explode and do random shit
    // Such that yaw stays within its bounds. To work around that I detect that behaviour and rotate R into
    // positive yaw territory. Then everything behaves fine, I take the eulerAngles and just have to manually
    // subtract that rotation just from yaw.
    if (euler(1) >= M_PI/2. || euler(1) <= -M_PI/2.) {
		euler = (AngleAxisd(M_PI, Vector3d::UnitZ()) * R).eulerAngles(2, 0, 1);
		euler(0) -= M_PI;
	}

    // Yaw
    euler(0) += constrain(input_roll_ * kMaxYawRate + input_yaw_ * kMaxYawRate / 5., -kMaxYawRate, kMaxYawRate) * d_sec;
    // Pitch
    euler(1) = constrain(input_pitch_ * kMaxPitch, -kMaxPitch, kMaxPitch);
    // Roll
    euler(2) = constrain(-input_roll_ * kMaxRoll, -kMaxRoll, kMaxRoll);
    // Dependency on Target Heading and speed can be added later to those controls
    // Reassemble Rotation
    R = AngleAxisd(euler(0), Vector3d::UnitZ()) * AngleAxisd(euler(1), Vector3d::UnitX()) * AngleAxisd(euler(2), Vector3d::UnitY());
    target_attitude_ = Quaterniond(R);
}

void FlightController::attitude_controls(double& roll, double& pitch, double& yaw) {
    const double recomp_threshold = 0.2;

    static bool is_K_set = false;
    static double last_v_rel = 0;
    static Vector3d last_ctrl_surf_params(0, 0, 0);
    static Vector3d last_ang_drift_params(0, 0, 0);
    static MatrixXd last_K = MatrixXd::Zero(3, 3);
    static Vector3d last_ctrl_0s(0, 0, 0);

    double v_rel;
    Vector3d ctrl_surf_params;
    Vector3d ang_drift_params;
    if (is_kf_setup_) {
        v_rel = utility_kf_.state_vector()(3);
        ctrl_surf_params = utility_kf_.state_vector()(seq(9, 11));
        ang_drift_params = utility_kf_.state_vector()(seq(12, 14));
    } else {  // Use hard coded default values if there is no estimator data available
        v_rel = kVRelDefault;
        double cPitchDefault = kCPitchDefault;  // Eigen doesn't work with constants apparently
        double cRollDefault = kCRollDefault;
        double cYawDefault = kCYawDefault;
        double w0PitchDefault = kW0PitchDefault;
        double w0RollDefault = kW0RollDefault;
        double w0YawDefault = kW0YawDefault;
        ctrl_surf_params = Vector3d(cPitchDefault, cRollDefault, cYawDefault);
        ang_drift_params = Vector3d(w0PitchDefault, w0RollDefault, w0YawDefault);
    }

    // The following block doesn't have to be recomputed at every step (Just save K and the three ctrl_0s)
    bool too_much_param_change = last_v_rel == 0. || abs(v_rel - last_v_rel)/last_v_rel > recomp_threshold ||
                                 last_ctrl_surf_params(0) == 0. || abs(ctrl_surf_params(0) - last_ctrl_surf_params(0))/last_ctrl_surf_params(0) > recomp_threshold ||
                                 last_ctrl_surf_params(1) == 0. || abs(ctrl_surf_params(1) - last_ctrl_surf_params(1))/last_ctrl_surf_params(1) > recomp_threshold ||
                                 last_ctrl_surf_params(2) == 0. || abs(ctrl_surf_params(2) - last_ctrl_surf_params(2))/last_ctrl_surf_params(2) > recomp_threshold;
                                 // The change in w0 is not taken into account because it will produce big relative changes without necessarily influencing controller behaviour
    if (!is_K_set || too_much_param_change) {
        // sqrt(r_0) = -w_0 / (c * v_rel)
        Vector3d sqrt_ctrl_0s = -ang_drift_params.cwiseQuotient(ctrl_surf_params * v_rel);
        Vector3d ctrl_0s = sqrt_ctrl_0s.unaryExpr(function<double(double)>(sgn_sq));

        // LQR
        MatrixXd A = MatrixXd::Zero(3, 3);
        MatrixXd B = (ctrl_surf_params * v_rel).asDiagonal();
        MatrixXd Q = Vector3d(2., 2., 2.).asDiagonal();  // LQR Controller tuning in here for now
        MatrixXd R = Vector3d(0.1, 0.1, 0.1).asDiagonal();

        MatrixXd K = lqr(A, B, Q, R);

        last_K = K;
        last_v_rel = v_rel;
        last_ctrl_surf_params = ctrl_surf_params;
        last_ang_drift_params = ang_drift_params;
        last_ctrl_0s = ctrl_0s;

        is_K_set = true;
    }  // Block that doesn't have to be recomputed every step ends here

    Quaterniond attitude = imu_.attitude();
    Quaterniond d_att = target_attitude_.conjugate() * attitude;
    // Only positive quaternions allowed for linearization
    if (d_att.w() < 0.)
        d_att = Quaterniond(-d_att.w(), -d_att.x(), -d_att.y(), -d_att.z());
    Vector3d d_theta(2.*d_att.x(), 2.*d_att.y(), 2.*d_att.z());
    
    Vector3d sqrt_ctrl = -last_K * d_theta;
    
    pitch = last_ctrl_0s(0) + sgn(sqrt_ctrl(0)) * sq(sqrt_ctrl(0));
    roll = last_ctrl_0s(1) + sgn(sqrt_ctrl(1)) * sq(sqrt_ctrl(1));
    yaw = last_ctrl_0s(2) + sgn(sqrt_ctrl(2)) * sq(sqrt_ctrl(2));
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
            U.col(j) = eigs.eigenvectors().col(i).real();
            ++j;
        }
    }

    MatrixXd U1 = U.topRows(n);
    MatrixXd U2 = U.bottomRows(n);
    MatrixXd P = U2 * U1.inverse();
    MatrixXd K = R_inv * B.transpose() * P;
    return K;
}