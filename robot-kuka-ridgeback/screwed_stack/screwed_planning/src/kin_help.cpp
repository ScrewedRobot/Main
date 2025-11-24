
#include "kin_help.h"


Vector6d jointToCartesianVelocity(const Vector6d joint_positions,
                                    const Vector6d joint_velocities)
{
    
    // const Matrix6x6d jacobian = calculateJacobian(joint_positions);

    Matrix6x4d dh_params;
    dh_params << -M_PI/2, 0.34, 0.0, 0.0,   // Joint 1
                  M_PI/2, 0.00, 0.0, 0.0, // Joint 2
                  M_PI/2, 0.40, 0.0, 0.0,   // Joint 3
                 -M_PI/2, 0.00, 0.0, 0.0, // Joint 4
                 -M_PI/2, 0.40, 0.0, 0.0,   // Joint 5
                  M_PI/2, 0.00, 0.0, 0.0; // Joint 6

    const Matrix6x6d jacobian = calculateJacobian(joint_positions);

    // Compute the cartesian velocity
    Vector6d cartesian_velocity = jacobian * joint_velocities;

    return cartesian_velocity;
}


// Standard DH to transformation matrix
Eigen::Matrix4d dh_to_matrix(double alpha, double d, double a, double theta) {
    double ca = cos(alpha), sa = sin(alpha);
    double ct = cos(theta), st = sin(theta);
    Eigen::Matrix4d A;
    A << ct, -st*ca,  st*sa, a*ct,
         st,  ct*ca, -ct*sa, a*st,
          0,     sa,     ca,    d,
          0,      0,      0,    1;
    return A;
}


Eigen::Matrix4d computeForwardKinematics(const Vector6d joint_angles, 
                                         const Matrix6x4d& dh_params)
{
    // This function computes the forward kinematics for a 6-DOF manipulator
    // using the Denavit-Hartenberg parameters.

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 6; ++i) {
        double alpha = dh_params(i, 0);
        double d = dh_params(i, 1);
        double a = dh_params(i, 2);
        double theta = joint_angles[i];

        T *= dh_to_matrix(alpha, d, a, theta);
    }

    return T;
}

Vector6d poseFromTransform(const Eigen::Matrix4d& T)
{
    Vector6d pose;
    // Position
    pose.head<3>() = T.block<3,1>(0,3);
    // Orientation (roll, pitch, yaw)
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2); // roll (X), pitch (Y), yaw (Z)
    pose.tail<3>() = rpy;
    return pose;
}


Matrix6x6d calculateNumericJacobian(const Vector6d joint_angles,
                                    const Matrix6x4d& dh_params)
{
    // This function computes the numerical Jacobian for a 6-DOF manipulator
    // using finite differences. It assumes that the joint angles are given in radians.
    
    Matrix6x6d jacobian;
    const double delta = 1e-6; // Small perturbation for numerical differentiation

    for (int i = 0; i < 6; ++i) {
        Vector6d perturbed_angles = joint_angles;
        perturbed_angles[i] += delta;

        Vector6d pose_plus = poseFromTransform(computeForwardKinematics(perturbed_angles, dh_params));
        
        perturbed_angles[i] -= 2 * delta; // Perturb in the negative direction
        Vector6d pose_minus = poseFromTransform(computeForwardKinematics(perturbed_angles, dh_params));

        jacobian.col(i) = (pose_plus - pose_minus) / (2 * delta);
    }

    return jacobian;
}

// Returns a 6x6 geometric Jacobian for a 6-DOF manipulator
Matrix6x6d calculateJacobian(const Vector6d q)
{

    const double c1 = cos(q[0]);
    const double s1 = sin(q[0]);
    const double c2 = cos(q[1]);
    const double s2 = sin(q[1]);
    const double c3 = cos(q[2]);
    const double s3 = sin(q[2]);
    const double c4 = cos(q[3]);
    const double s4 = sin(q[3]);
    const double c5 = cos(q[4]);
    const double s5 = sin(q[4]);
    const double c6 = cos(q[5]);
    const double s6 = sin(q[5]);

    
    Matrix6x6d jacobian = Matrix6x6d::Zero();


    // Row 0 - Linear velocity in x
    jacobian(0, 0) = -0.4*c4*s1*s2 - 0.4*s1*s2 + 0.4*s4*(c1*s3 + c2*c3*s1);
    jacobian(0, 1) = 0.4*c1*(c2*c4 + c2 + c3*s2*s4);
    jacobian(0, 2) = 0.4*s4*(c1*c2*s3 + c3*s1);
    jacobian(0, 3) = -0.4*c1*c2*c3*c4 - 0.4*c1*s2*s4 + 0.4*c4*s1*s3;

    // Row 1 - Linear velocity in y
    jacobian(1, 0) = 0.4*c1*c4*s2 + 0.4*c1*s2 + 0.4*s4*(-c1*c2*c3 + s1*s3);
    jacobian(1, 1) = 0.4*s1*(c2*c4 + c2 + c3*s2*s4);
    jacobian(1, 2) = 0.4*s4*(-c1*c3 + c2*s1*s3);
    jacobian(1, 3) = -0.4*c1*c4*s3 - 0.4*c2*c3*c4*s1 - 0.4*s1*s2*s4;

    // Row 2 - Linear velocity in z
    jacobian(2, 1) = 0.4*c2*c3*s4 - 0.4*c4*s2 - 0.4*s2;
    jacobian(2, 2) = -0.4*s2*s3*s4;
    jacobian(2, 3) = -0.4*c2*s4 + 0.4*c3*c4*s2;

    // Row 3 - Angular velocity in roll
    jacobian(3, 1) = -s1;
    jacobian(3, 2) = c1*s2;
    jacobian(3, 3) = c1*c2*s3 + c3*s1;
    jacobian(3, 4) = c1*c4*s2 + s4*(-c1*c2*c3 + s1*s3);
    jacobian(3, 5) = -c5*(c1*c2*s3 + c3*s1) + s5*(-c1*s2*s4 + c4*(-c1*c2*c3 + s1*s3));

    // Row 4 - Angular velocity in pitch
    jacobian(4, 1) = c1;
    jacobian(4, 2) = s1*s2;
    jacobian(4, 3) = -c1*c3 + c2*s1*s3;
    jacobian(4, 4) = c4*s1*s2 - s4*(c1*s3 + c2*c3*s1);
    jacobian(4, 5) = -c5*(-c1*c3 + c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4);

    // Row 5 - Angular velocity in yaw
    jacobian(5, 0) = 1.0;
    jacobian(5, 2) = c2;
    jacobian(5, 3) = -s2*s3;
    jacobian(5, 4) = c2*c4 + c3*s2*s4;
    jacobian(5, 5) = c5*s2*s3 + s5*(-c2*s4 + c3*c4*s2);
    
    return jacobian;
}


Vector6d dampedLeastSquares(const Matrix6x6d& J, const Vector6d& x_dot, double lambda = 1e-2)
{
    Matrix6x6d JT = J.transpose();
    Matrix6x6d damping = lambda * lambda * Matrix6x6d::Identity();
    Matrix6x6d A = JT * J + damping; // Damped least squares matrix

    Vector6d q_dot = A.ldlt().solve(JT * x_dot);
    return q_dot;
}

Matrix6x6d dampedPinv(const Matrix6x6d& J, double lambda = 1e-2)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto s = svd.singularValues();
    auto U = svd.matrixU();
    auto V = svd.matrixV();

    Eigen::VectorXd d = s.array() / (s.array().square() + lambda * lambda);
    Eigen::MatrixXd D = d.asDiagonal();

    return V * D * U.transpose();
}

Matrix6x6d svdPinv(const Matrix6x6d& J, double tolerance = 1e-15)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto s = svd.singularValues();
    auto U = svd.matrixU();
    auto V = svd.matrixV();

    // Eigen::VectorXd s_inv = s.unaryExpr(
    //     [&](double sigma){ return sigma>tol ? 1.0/sigma : 0.0;}
    // );

    // Eigen::VectorXd s_inv = s.array().inverse();

    double eps = std::numeric_limits<double>::epsilon();
    // Relative tol = eps * max(dim) * largest singular value
    double tol = eps * std::max(J.rows(), J.cols()) * s(0);
 
    Eigen::VectorXd s_inv(s.size());
    for(int i = 0; i < s.size(); ++i) {
        double sigma = s(i);
        // if sigma is NaN or too small, zero it out:
        if (!std::isfinite(sigma) || std::abs(sigma) <= tol) {
            s_inv(i) = 0.0;
        } else {
            s_inv(i) = 1.0 / sigma;
        }
    }
    
    return V * s_inv.asDiagonal() * U.transpose();
}