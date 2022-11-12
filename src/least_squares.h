#ifndef LEAST_SQUARES_H_
#define LEAST_SQUARES_H_

#include <Arduino.h>
#include <ArduinoEigen.h>

using namespace std;
using namespace Eigen;

// Iterative Least Squares
class LeastSquares {
public:
    static Matrix2d givens(double a, double b);

    LeastSquares();
    LeastSquares(const MatrixXd& A, const VectorXd& b);
    LeastSquares(const LeastSquares& ls);
    void set(const MatrixXd& A, const VectorXd& b);
    int rows() const;
    bool is_solvable() const;
    VectorXd solve();
    bool recompute_qr();
    void add_row(const VectorXd& a, double b);
    void remove_n_rows(int n);

private:
    bool is_A_set_;
    bool is_A_full_rank_;
    bool is_last_x_valid_;

    MatrixXd A_;
    VectorXd b_;
    VectorXd x_;  // Solution

    PermutationMatrix<Dynamic> P_;
    MatrixXd Q_;
    MatrixXd R_;
};

#endif  // LEAST_SQUARES_H_