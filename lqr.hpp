// Author Akash Patel (apatel435@gatech.edu)
//
// Purpose: Helper source code file for determining optimal gain matrix using lqr
// Returns true if successful and outputs caluclated gain matrix in input
// pointer

// Includes
#include <dart/dart.hpp>

// Function Prototypes

/**
 * @brief Computes the LQR gain matrix (usually denoted K) for a discrete time
 * infinite horizon problem.
 *
 * @param a State matrix of the underlying system
 * @param b Input matrix of the underlying system
 * @param q Weight matrix penalizing the state
 * @param r Weight matrix penalizing the controls
 * @param n Weight matrix penalizing state / control pairs
 * @param k Pointer to the generated matrix (has to be a double/dynamic size
 * matrix!)
 */

// // LQR Method (without n)
bool lqr(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd q, Eigen::MatrixXd r, Eigen::MatrixXd *k);

// // LQR Method (full)
bool lqr(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd q, Eigen::MatrixXd r, Eigen::MatrixXd n, Eigen::MatrixXd *k);
