#include "kalman_filter.h"

KalmanFilter::KalmanFilter(function<VectorXd(VectorXd, VectorXd)> system_model,
                           function<VectorXd(VectorXd)> measure_model,
                           function<MatrixXd(VectorXd, VectorXd)> system_jacobian,
                           function<MatrixXd(VectorXd)> measure_jacobian,
                           MatrixXd noise_cov, MatrixXd measure_cov) {
    system_model_ = system_model;
    measure_model_ = measure_model;
    system_jacobian_ = system_jacobian;
    measure_jacobian_ = measure_jacobian;
    noise_cov_ = noise_cov;
    measure_cov_ = measure_cov;
}

void KalmanFilter::setup(VectorXd init_state, MatrixXd init_err_cov) {
    state_ = init_state;
    error_cov_ = init_err_cov;
}

void KalmanFilter::propagate(VectorXd input, double dt) {
    auto state_dot = [this, input](VectorXd x){ return system_model_(x, input); };
    auto cov_dot = [this, input](MatrixXd P) {
        return system_jacobian_(state_, input) * P + P * system_jacobian_(state_, input).transpose() + noise_cov_;
    };

    // Warning! Integration for Error Covariance uses fixed state instead
    // of updating it inside of the integration scheme. This may lead to
    // numerical instabilities because of inconsistend integration.
    // Could be solved by combining state and covariance into one object
    // that is integrated as one.
    VectorXd new_state = ode::rk4(state_, state_dot, dt);
    MatrixXd new_cov = ode::rk4(error_cov_, cov_dot, dt);

    state_ = new_state;
    error_cov_ = new_cov;  // Might lose its symmetry over time. Restore if severe
}

void KalmanFilter::update(VectorXd measurement) {
    ;
}