#ifndef ODE_H_
#define ODE_H_

namespace ode {

    // Euler Forward integration step
    template <typename Vector, typename Function>
    Vector euler(const Vector& x_0, Function f, double dt) {
        Vector x_dot = f(x_0);
        Vector x_1 = x_0 + dt * x_dot;
        return x_1;
    }

    // Euler Forward step with trapezoidal corrector step
    template <typename Vector, typename Function>
    Vector euler_trapezoid(const Vector& x_0, Function f, double dt) {
        Vector x_dot_0 = f(x_0);
        Vector x_11 = x_0 + x_dot_0 * dt;
        Vector x_dot_11 = f(x_11);
        Vector x_12 = x_0 + 0.5 * dt * (x_dot_0 + x_dot_11);
        return x_12;
    }

    // Runge Kutta Second order step
    template <typename Vector, typename Function>
    Vector rk2(const Vector& x_0, Function f, double dt) {
        Vector x_dot_0 = f(x_0);
        Vector x_01 = x_0 + 0.5 * dt * x_dot_0;
        Vector x_dot_01 = f(x_01);
        Vector x_1 = x_0 + x_dot_01 * dt;
        return x_1;
    }

    // Classical Runge Kutta fourth order step
    template <typename Vector, typename Function>
    Vector rk4(const Vector& x_0, Function f, double dt) {
        static const double kSixth = 1. / 6.;
        Vector k_1 = f(x_0);
        Vector k_2 = f(x_0 + 0.5 * dt * k_1);
        Vector k_3 = f(x_0 + 0.5 * dt * k_2);
        Vector k_4 = f(x_0 + dt * k_3);
        Vector x_1 = x_0 + 1. / 6. * dt * (k_1 + 2. * k_2 + 2. * k_3 + k_4);
        return x_1;
    }

};

#endif  // ODE_H_