#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <Arduino.h>
#include <ArduinoEigen.h>
#include "ode.h"

using namespace std;
using namespace Eigen;

class KalmanFilter {
public:
    struct State {
        VectorXd state_;
        MatrixXd error_cov_;
    };

    KalmanFilter(function<VectorXd(VectorXd, VectorXd)> system_model,
                 function<VectorXd(VectorXd)> measure_model,
                 function<MatrixXd(VectorXd, VectorXd)> system_jacobian,
                 function<MatrixXd(VectorXd)> measure_jacobian,
                 MatrixXd noise_cov, MatrixXd measure_cov);
    void setup(VectorXd init_state, MatrixXd init_err_cov);
    void propagate(VectorXd input, double dt);
    void update(VectorXd measurement);
    VectorXd state() const { return state_; }
    MatrixXd error_cov() const { return error_cov_; }

private:
    VectorXd state_;
    MatrixXd error_cov_;    // Error Covariance
    MatrixXd noise_cov_;    // Continuous Noise Covariance
    MatrixXd measure_cov_;  // Measurement Covariance

    function<VectorXd(VectorXd, VectorXd)> system_model_;     // x_dot = f(x, u)
    function<VectorXd(VectorXd)> measure_model_;              // z = h(x)
    function<MatrixXd(VectorXd, VectorXd)> system_jacobian_;  // df/dx
    function<MatrixXd(VectorXd)> measure_jacobian_;           // dh/dx
};

#endif  // KALMAN_FILTER_H_