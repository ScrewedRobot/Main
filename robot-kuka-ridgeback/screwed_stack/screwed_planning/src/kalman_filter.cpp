#include "kalman_filter.h"


JointKalmanFilter::JointKalmanFilter(
        float fs,                    // Sampling frequency
        Eigen::VectorXd sigma_a,     // Process noise standard deviation
        Eigen::VectorXd sigma_q,     // Measurement noise standard deviation
        Eigen::VectorXd sigma_qdot,  // Measurement noise standard deviation for joint velocities
        int n_joints)                 // Number of joints
{
    dt_ = 1.0 / fs;
    n_states_ = 2 * n_joints;

    x_ = Eigen::VectorXd::Zero(n_states_); // State vector [q, qdot]
    P_ = Eigen::MatrixXd::Identity(n_states_, n_states_); // State covariance matrix

    // State transition matrix F
    I_ = Eigen::MatrixXd::Identity(n_joints, n_joints);
    F_ = Eigen::MatrixXd::Zero(n_states_, n_states_);
    F_.block(0, 0, n_joints, n_joints) = I_;
    F_.block(0, n_joints, n_joints, n_joints) = dt_ * I_;
    F_.block(n_joints, n_joints, n_joints, n_joints) = I_;

    H_ = Eigen::MatrixXd::Identity(n_states_, n_states_);

    // Process noise covariance matrix Q
    Eigen::MatrixXd G(2 * n_joints, n_joints);
    G << 0.5 * dt_ * dt_ * I_, 
        dt_ * I_;
    Q_ = G * sigma_a.cwiseProduct(sigma_a).asDiagonal() * G.transpose();

    // Measurement noise covariance matrix
    Eigen::VectorXd q_m = sigma_q.array().square();
    Eigen::VectorXd qdot_m = sigma_qdot.array().square();
    Eigen::VectorXd M(q_m.size() + qdot_m.size());
    M << q_m, qdot_m;
    R_ = M.asDiagonal(); 

}

Eigen::VectorXd JointKalmanFilter::process(const Eigen::VectorXd& z)
{
    // Predict step
    predict();

    // Update step
    update(z);

    return x_;
}

void JointKalmanFilter::predict()
{
    // Predict the next state
    x_ = F_ * x_;
    
    // Update the state covariance
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void JointKalmanFilter::update(const Eigen::VectorXd& z)
{
    // Compute the Kalman gain
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // Update the state estimate
    x_ = x_ + K * (z - H_ * x_);

    // Update the state covariance
    P_ = (Eigen::MatrixXd::Identity(n_states_, n_states_) - K * H_) * P_;
}
