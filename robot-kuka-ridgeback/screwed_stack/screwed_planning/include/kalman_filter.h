
#pragma once

#include <Eigen/Dense>


class JointKalmanFilter
{
public:
    JointKalmanFilter(
        float fs,                    // Sampling frequency
        Eigen::VectorXd sigma_a,     // Process noise standard deviation
        Eigen::VectorXd sigma_q,     // Measurement noise standard deviation
        Eigen::VectorXd sigma_qdot,  // Measurement noise standard deviation for joint velocities
        int n_joints                 // Number of joints
    );

    Eigen::VectorXd process(const Eigen::VectorXd& z);


private:

    void predict();

    void update(const Eigen::VectorXd& z);

    Eigen::VectorXd x_; // State vector [q, qdot]
    Eigen::MatrixXd P_; // State covariance matrix
    Eigen::MatrixXd Q_; // Process noise covariance matrix
    Eigen::MatrixXd R_; // Measurement noise covariance matrix
    Eigen::MatrixXd F_; // State transition matrix
    Eigen::MatrixXd H_; // Measurement matrix
    Eigen::MatrixXd I_; // Identity matrix for updates

    double dt_; // Time step
    int n_states_; // Number of states (2 * n_joints)
    
};