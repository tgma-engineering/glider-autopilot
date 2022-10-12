#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <functional>
#include <ArduinoEigen.h>
#include "ode.h"

using namespace std;
using namespace Eigen;

// Extended Kalman Filter implementation using Eigen for Linear Algebra
class KalmanFilter {
public:
    // Struct to save state vector and error covariance of Kalman Kilter in one object.
    // This allows it to be integrated in one ode-solver function call.
    struct State {
        VectorXd state_vector_;
        MatrixXd error_cov_;

        State() : state_vector_(), error_cov_() {}
        State(const VectorXd& state_vector, const MatrixXd& error_cov) : state_vector_(state_vector), error_cov_(error_cov) {}
        State(const State& state) : state_vector_(state.state_vector_), error_cov_(state.error_cov_) {}
        State& operator=(const State& state) {
            if (&state == this)
                return *this;
            state_vector_ = state.state_vector_;
            error_cov_ = state.error_cov_;
            return *this;
        }
        State operator-(const State& state) {
            State temp_state;
            temp_state.state_vector_ = -state.state_vector_;
            temp_state.error_cov_ = -state.error_cov_;
            return temp_state;
        }
    };

    KalmanFilter(function<VectorXd(const VectorXd&, const VectorXd&)> system_model,
        function<VectorXd(const VectorXd&)> measure_model,
        function<MatrixXd(const VectorXd&, const VectorXd&)> system_jacobian,
        function<MatrixXd(const VectorXd&)> measure_jacobian,
        const MatrixXd& noise_cov, const MatrixXd& measure_cov);
    void setup(const VectorXd& init_state, const MatrixXd& init_err_cov);
    void propagate(const VectorXd& input, double dt);
    void update(const VectorXd& measurement);
    VectorXd state_vector() const { return state_.state_vector_; }
    MatrixXd error_cov() const { return state_.error_cov_; }

private:
    State state_;           // State vector and error covariance (combined for integration purposes)
    MatrixXd noise_cov_;    // Continuous Noise Covariance
    MatrixXd measure_cov_;  // Measurement Covariance

    function<VectorXd(const VectorXd&, const VectorXd&)> system_model_;     // x_dot = f(x, u)
    function<VectorXd(const VectorXd&)> measure_model_;                     // z = h(x)
    function<MatrixXd(const VectorXd&, const VectorXd&)> system_jacobian_;  // df/dx
    function<MatrixXd(const VectorXd&)> measure_jacobian_;                  // dh/dx
};

#endif  // KALMAN_FILTER_H_