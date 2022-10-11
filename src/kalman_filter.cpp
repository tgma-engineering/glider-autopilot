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
    return state * (1./d);
}

KalmanFilter::State operator/(double d, const KalmanFilter::State& state) {
    return state * (1./d);
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
    auto state_dot = [this, input](const VectorXd& x){ return system_model_(x, input); };
    auto cov_dot = [this, input](const MatrixXd& P) {
        return system_jacobian_(state_.state_vector_, input) * P + P * system_jacobian_(state_.state_vector_, input).transpose() + noise_cov_;
    };

    // Warning! Integration for Error Covariance uses fixed state instead
    // of updating it inside of the integration scheme. This may lead to
    // numerical instabilities because of inconsistend integration.
    // Could be solved by combining state and covariance into one object
    // that is integrated as one.
    VectorXd new_state = ode::rk4(state_.state_vector_, state_dot, dt);
    MatrixXd new_cov = ode::rk4(state_.error_cov_, cov_dot, dt);

    state_.state_vector_ = new_state;
    state_.error_cov_ = new_cov;  // Might lose its symmetry over time. Restore if severe
}

void KalmanFilter::update(const VectorXd& measurement) {
    ;
}