// Author Munzir Zafar (mzafar7@gatech.edu)
//
// Purpose: Helper source code file for determining optimal gain matrix using lqr method (continuous time)
// LQR Solver for Continuous Time Infinite Horizon Problem
// Returns true if successful and outputs calculated gain matrix in input
// pointer

// Includes
#include <dart/dart.hpp>

#include "lqr.hpp"

// Namespaces
using namespace std;

// Function
// // LQR method (without n)
bool lqr(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd q, Eigen::MatrixXd r, Eigen::MatrixXd *k) {
    Eigen::MatrixXd n = Eigen::MatrixXd::Zero(a.rows(), b.cols());

    return lqr(a, b, q, r, n, k);
}

// // LQR Method (full)
bool lqr(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd q, Eigen::MatrixXd r, Eigen::MatrixXd n, Eigen::MatrixXd *k) {
    // Check dimensionality of input matrices

    // Check a
    if (a.rows() != a.cols()) {
        cout << "LQR Error: First (a) matrix must be square!" << endl;
        return false;
    }

    // Check b
    if (b.rows() != a.cols()) {
        cout << "LQR Error: First (a) and second (b) matrix must be conformal!" << endl;
        return false;
    }

    // Check q
    if (q.rows() != q.cols() || q.rows() != a.cols()) {
        cout << "LQR Error: Third (q) matrix must be square and conformal with the first (a) matrix!" << endl;
        return false;
    }

    // Check r
    if (r.rows() != r.cols() || r.cols() != b.cols()) {
        cout << "LQR Error: Fourth (r) matrix must be square and conformal with column dimension of second (b) matrix!" << endl;
        return false;
    }

    // Check n
    if (n.rows() != b.rows() || n.cols() != b.cols()) {
        cout << "LQR Error: Fifth (n) matrix must be identically dimensioned with second (b) matrix!" << endl;
        return false;
    }

    // TODO add Munzir's stuff

    return true;
}
