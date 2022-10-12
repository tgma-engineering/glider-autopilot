#include "kalman_filter.h"

KalmanFilter::State operator*(const KalmanFilter::State& state, double d) {
    KalmanFilter::State temp_state;
    temp_state.state_vector_ = state.state_vector_ * d;
    temp_state.error_cov_ = state.error_cov_ * d;
    return temp_state;
}

KalmanFilter::State operator*(double d, const KalmanFilter::State& state) {
    return state * d;
}

KalmanFilter::State operator/(const KalmanFilter::State& state, double d) {
    return state * (1. / d);
}

KalmanFilter::State operator/(double d, const KalmanFilter::State& state) {
    return state * (1. / d);
}

KalmanFilter::State operator+(const KalmanFilter::State& lhs, const KalmanFilter::State& rhs) {
    KalmanFilter::State temp_state;
    temp_state.state_vector_ = lhs.state_vector_ + rhs.state_vector_;
    temp_state.error_cov_ = lhs.error_cov_ + rhs.error_cov_;
    return temp_state;
}

KalmanFilter::State operator-(const KalmanFilter::State& lhs, const KalmanFilter::State& rhs) {
    KalmanFilter::State temp_state;
    temp_state.state_vector_ = lhs.state_vector_ - rhs.state_vector_;
    temp_state.error_cov_ = lhs.error_cov_ - rhs.error_cov_;
    return temp_state;
}

KalmanFilter::KalmanFilter(function<VectorXd(const VectorXd&, const VectorXd&)> system_model,
    function<VectorXd(const VectorXd&)> measure_model,
    function<MatrixXd(const VectorXd&, const VectorXd&)> system_jacobian,
    function<MatrixXd(const VectorXd&)> measure_jacobian,
    const MatrixXd& noise_cov, const MatrixXd& measure_cov) {
    system_model_ = system_model;
    measure_model_ = measure_model;
    system_jacobian_ = system_jacobian;
    measure_jacobian_ = measure_jacobian;
    noise_cov_ = noise_cov;
    measure_cov_ = measure_cov;
}

void KalmanFilter::setup(const VectorXd& init_state, const MatrixXd& init_err_cov) {
    state_.state_vector_ = init_state;
    state_.error_cov_ = init_err_cov;
}

void KalmanFilter::propagate(const VectorXd& input, double dt) {
    auto state_dot = [this, input](const State& s) -> State {
        State s_dot;
        VectorXd model = system_model_(s.state_vector_, input);
        MatrixXd jacobian = system_jacobian_(s.state_vector_, input);
        s_dot.state_vector_ = model;
        s_dot.error_cov_ = jacobian * s.error_cov_ + s.error_cov_ * jacobian.transpose() + noise_cov_;
        return s_dot;
    };

    State new_state = ode::rk4(state_, state_dot, dt);  // Error Covariance might lose its symmetry over time
    state_ = new_state;
}

void KalmanFilter::update(const VectorXd& measurement) {
    auto n = state_.state_vector_.size();  // State size
    auto I_n = MatrixXd::Identity(n, n);   // Square Identity of state size

    MatrixXd meas_jac = measure_jacobian_(state_.state_vector_);
    // Kalman Gain Matrix
    MatrixXd K = state_.error_cov_ * meas_jac.transpose() * (meas_jac * state_.error_cov_ * meas_jac.transpose() + measure_cov_).inverse();

    State state_update;
    state_update.state_vector_ = state_.state_vector_ + K * (measurement - measure_model_(state_.state_vector_));
    state_update.error_cov_ = (I_n - K * meas_jac) * state_.error_cov_;  // Simple update, bad numerical stability and symmetry properties known

    state_ = state_update;
}