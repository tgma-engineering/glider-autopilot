#include "least_squares.h"

// Computes Givens Rotation matrix R such that R.T * (a, b).T = (x, 0).T
Matrix2d LeastSquares::givens(double a, double b) {
    double c, s;
    if (b == 0.) {
        c = 1.;
        s = 0.;
    } else {
        if (abs(b) < abs(a)) {
            double t = -a / b;
            s = 1 / sqrt(1 + t*t);
            c = s * t;
        } else {
            double t = -b / a;
            c = 1 / sqrt(1 + t*t);
            s = c * t;
        }
    }
    return Matrix2d{{c, s}, {-s, c}};
}

LeastSquares::LeastSquares() {
    is_A_set_ = false;
    is_A_full_rank_ = false;
    is_last_x_valid_ = false;
}

// Object for least squares system Ax = b
// A has to be tall and skinny (at least a little bit)
LeastSquares::LeastSquares(const MatrixXd& A, const VectorXd& b) {
    A_ = A;
    b_ = b;

    is_A_set_ = true;
    is_A_full_rank_ = recompute_qr();
    is_last_x_valid_ = false;
}

LeastSquares::LeastSquares(const LeastSquares& ls) {
    is_A_set_ = ls.is_A_set_;
    is_A_full_rank_ = ls.is_A_full_rank_;
    is_last_x_valid_ = ls.is_last_x_valid_;

    A_ = ls.A_;
    b_ = ls.b_;
    x_ = ls.x_;

    P_ = ls.P_;
    Q_ = ls.Q_;
    R_ = ls.R_;
}

void LeastSquares::set(const MatrixXd& A, const VectorXd& b) {
    A_ = A;
    b_ = b;

    is_A_set_ = true;
    is_A_full_rank_ = recompute_qr();
    is_last_x_valid_ = false;
}

int LeastSquares::rows() const {
    if (is_A_set_) {
        return A_.rows();
    } else {
        return 0;
    }
}

bool LeastSquares::is_solvable() const {
    if (is_A_full_rank_) {
        return true;
    } else {
        return false;
    }
}

VectorXd LeastSquares::solve() {
    if (!is_A_set_) {
        Serial.println("Error: Matrix A is undefined");
        return VectorXd::Zero(1);
    }

    if (is_A_full_rank_) {
        if (is_last_x_valid_) {
            return x_;
        } else {
            VectorXd x_ = P_.transpose() * R_.triangularView<Upper>().solve(Q_.transpose() * b_);
            is_last_x_valid_ = true;
            return x_;
        }
    } else {
        Serial.println("Error: Matrix A does not have full column rank");
        return VectorXd::Zero(1);
    }
}

// Returns if A has full column rank
bool LeastSquares::recompute_qr() {
    is_last_x_valid_ = false;
    if (!is_A_set_) {
        Serial.println("Error: Matrix A is undefined");
        return false;
    }
    if (A_.rows() >= A_.cols()) {
        ColPivHouseholderQR<MatrixXd> qr = A_.colPivHouseholderQr();
        P_ = qr.colsPermutation();
        Q_ = qr.householderQ().setLength(qr.nonzeroPivots());
        R_ = qr.matrixR().template triangularView<Upper>();
        return A_.cols() == qr.rank();
    } else {
        return false;
    }
}

// Insert one row in the beginning of A and one element in the beginning of b
void LeastSquares::add_row(const VectorXd& a, double b) {
    is_last_x_valid_ = false;
    // If A is undefined create A with just one row
    if (!is_A_set_) {
        int c = a.size();

        A_ = MatrixXd(1, c);
        A_(0, seq(0, c-1)) = a;
        b_ = VectorXd(1);
        b_ << b;
        
        is_A_set_ = true;
        is_A_full_rank_ = recompute_qr();
    } else {
        int A_r = A_.rows();
        int A_c = A_.cols();

        MatrixXd new_A(A_r+1, A_c);
        new_A(0, seq(0, A_c-1)) = a;
        new_A(seq(1, A_r), seq(0, A_c-1)) = A_;
        VectorXd new_b(A_r+1);
        new_b << b, b_;

        A_ = new_A;
        b_ = new_b;

        // If A didn't have full column rank before, check if it has now
        if (!is_A_full_rank_) {
            is_A_full_rank_ = recompute_qr();
            return;
        }

        // Compute Q and R
        MatrixXd new_R(A_r+1, A_c);
        new_R(0, seq(0, A_c-1)) = a.transpose() * P_;
        new_R(seq(1, A_r), seq(0, A_c-1)) = R_;
        // R is upper Hessenberg right now
        MatrixXd new_Q = MatrixXd::Identity(A_r+1, A_r+1);
        new_Q(seq(1, A_r), seq(1, A_r)) = Q_;

        for (int i = 0; i < A_c; ++i) {
            // Generate Givens rotation to remove one blow-diagonal element of new_R
            Matrix2d J = givens(new_R(i, i), new_R(i+1, i));

            // Apply Givens rotation to new_R from the left: new_R = J.T * new_R
            new_R(seq(i, i+1), seq(i, A_c-1)) = J.transpose() * new_R(seq(i, i+1), seq(i, A_c-1));
            
            // Apply Givens rotation to new_Q from the right: new_Q = new_Q * J
            new_Q(seq(0, A_r), seq(i, i+1)) = new_Q(seq(0, A_r), seq(i, i+1)) * J;
        }

        Q_ = new_Q;
        R_ = new_R;
        // Permutation matrix P stays unchanged
        // Might lead to stability problems if recompute_qr() isnt called here and there
    }
}

// Remove last n rows from A and b. Recomputes QR automatically
void LeastSquares::remove_n_rows(int n) {
    is_last_x_valid_ = false;

    if (n <= 0) {
        return;
    }

    int r = A_.rows();
    int c = A_.cols();

    int m = n;
    if (r <= n)
        m = r-1;

    MatrixXd new_A = A_(seq(0, r-1-m), seq(0, c-1));
    VectorXd new_b = b_(seq(0, r-1-m));

    A_ = new_A;
    b_ = new_b;
    
    is_A_full_rank_ = recompute_qr();
}