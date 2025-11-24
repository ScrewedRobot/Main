
#pragma once
#include <Eigen/Dense>
#include <vector>



typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 4> Matrix6x4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6x6d;



Vector6d jointToCartesianVelocity(const Vector6d joint_positions, const Vector6d joint_velocities);

Eigen::Matrix4d dh_to_matrix(double alpha, double d, double a, double theta);

Eigen::Matrix4d computeForwardKinematics(const Vector6d joint_angles, const Matrix6x4d& dh_params);

Vector6d poseFromTransform(const Eigen::Matrix4d& T);

Matrix6x6d calculateNumericJacobian(const Vector6d joint_angles, const Matrix6x4d& dh_params);

Matrix6x6d calculateJacobian(const Vector6d q);

Vector6d dampedLeastSquares(const Matrix6x6d& J, const Vector6d& x_dot, double lambda);

Matrix6x6d dampedPinv(const Matrix6x6d& J, double lambda);

Matrix6x6d svdPinv(const Matrix6x6d& J, double tol);








