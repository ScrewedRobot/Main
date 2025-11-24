

#include <ros/ros.h>
#include <Eigen/Dense>

#include "kin_help.h"
#include "iir_filter.h"
#include "fir_filter.h"
#include "kalman_filter.h"

#include <geometry_msgs/TwistStamped.h>
#include <iiwa_msgs/CartesianWrench.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointVelocity.h>
#include <screwed_msgs/DesiredForce.h>


class MotionServer
{
public:
  MotionServer(ros::NodeHandle& nh)
    : sampling_rate_(500.0),
      cutoff_freq_(20.0),
      kalman_filter_(500.0, 
        Eigen::VectorXd::Constant(7, 0.005),  // Process noise std dev
        Eigen::VectorXd::Constant(7, 0.0001),  // Measurement noise std dev
        Eigen::VectorXd::Constant(7, 0.1),  // Measurement noise std dev for joint velocities
        7) // Number of joints
  {

    // Publishers
    joint_pub_      = nh.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 10);
    cart_state_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/iiwa/state/CartesianVelocity", 10);
    filt_cart_state_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/iiwa/state/CartesianVelocityFiltered", 10);
    joint_vel_pub_ = nh.advertise<iiwa_msgs::JointVelocity>("/iiwa/state/JointVelocityTest", 10);

    // Subscribers
    joint_state_sub_ = nh.subscribe("/iiwa/state/JointPositionVelocity", 10, &MotionServer::jointStateCB, this);
    cart_vel_sub_ = nh.subscribe("/iiwa/command/CartesianVelocity", 10, &MotionServer::cartVelCB, this);
   

    // initialize butterworth filter

    filters_.resize(6); // 6 velocity components
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) { // 4th order
            filters_[i].emplace_back(sampling_rate_, cutoff_freq_);
        }
    }

    // initialize FIR filter

    // fir_filters_.resize(6); // 6 velocity components
    // for (int i = 0; i < 6; ++i) {
    //     fir_filters_[i] = FIRFilter();
    // }


    // initalize SG filter (Also FIR)

    sg_filters_.clear();
    for (int i = 0; i < 6; ++i)
        sg_filters_.emplace_back(51, 3, 0);

    // Initialize Kalman Filter with appropriate parameters
    Eigen::VectorXd sigma_a;
    sigma_a.resize(7);
    sigma_a << 2.85, 2.85, 2.91,
               3.78, 4.07, 5.24, 5.24;
    float alpha = 0.005;
    kalman_filter_ = JointKalmanFilter(1000.0, 
        sigma_a * alpha, // Process noise std dev
        Eigen::VectorXd::Constant(7, 0.0001),  // Measurement noise std dev
        Eigen::VectorXd::Constant(7, 0.1),  // Measurement noise std dev for joint velocities
        7); // Number of joints


    ROS_INFO("Direct Motion Planning Server is ready.");
  }

private:


  void cartVelCB(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    Vector6d x_dot;
    x_dot << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
                

    Vector6d q_dot = dampedLeastSquares(jacobian_, x_dot, 1e-2);

    iiwa_msgs::JointVelocity joint_msg;
    joint_msg.header = msg->header;
    joint_msg.velocity.a1 = q_dot[0];
    joint_msg.velocity.a2 = q_dot[1];
    joint_msg.velocity.a3 = q_dot[2];
    joint_msg.velocity.a4 = q_dot[3];
    joint_msg.velocity.a5 = q_dot[4];
    joint_msg.velocity.a6 = q_dot[5];
    joint_msg.velocity.a7 = 0.0; // Not used   

    joint_pub_.publish(joint_msg);
  }


  void jointStateCB(const iiwa_msgs::JointPositionVelocity::ConstPtr& msg)
  {
    // get cartesian velocity from joint

    Vector6d joint_pos;
    joint_pos << msg->position.a1, msg->position.a2, msg->position.a3,
                 msg->position.a4, msg->position.a5, msg->position.a6;

    Vector6d joint_vel;
    joint_vel << msg->velocity.a1, msg->velocity.a2, msg->velocity.a3,
                 msg->velocity.a4, msg->velocity.a5, msg->velocity.a6;

    
    // Kalman filter processing
    Eigen::VectorXd state_vector(14);
    state_vector << joint_pos, msg->position.a7, joint_vel, msg->velocity.a7; // [q, qdot]
    state_vector = kalman_filter_.process(state_vector);


    // --- Differentiate joint positions --- ( just testing a theory :) )
    Vector6d diff_joint_vel = Vector6d::Zero();
    if (has_prev_joint_state_) {
        double dt = (msg->header.stamp - prev_joint_time_).toSec();
        if (dt > 0.0) {
            diff_joint_vel = (joint_pos - prev_joint_pos_) / dt;

            // Publish differentiated velocities
            iiwa_msgs::JointVelocity diff_vel_msg;
            diff_vel_msg.header = msg->header;
            diff_vel_msg.velocity.a1 = diff_joint_vel[0];
            diff_vel_msg.velocity.a2 = diff_joint_vel[1];
            diff_vel_msg.velocity.a3 = diff_joint_vel[2];
            diff_vel_msg.velocity.a4 = diff_joint_vel[3];
            diff_vel_msg.velocity.a5 = diff_joint_vel[4];
            diff_vel_msg.velocity.a6 = diff_joint_vel[5];
            diff_vel_msg.velocity.a7 = 0.0;
            joint_vel_pub_.publish(diff_vel_msg);
        }
    }
    prev_joint_pos_ = joint_pos;
    prev_joint_time_ = msg->header.stamp;
    has_prev_joint_state_ = true;

    joint_vel = diff_joint_vel; // DEBUG: Use differentiated velocities, note this is a "BAD" way to do this

    geometry_msgs::TwistStamped twist, twist_filtered;
    twist_filtered.header = msg->header;
    twist_filtered.header.stamp = msg->header.stamp - ros::Duration(24.0 / sampling_rate_);
    twist.header = msg->header;

    jacobian_ = calculateJacobian(joint_pos);
    Vector6d cartesian_vel = jointToCartesianVelocity(joint_pos, joint_vel);


    // Apply IIR filtering to the cartesian velocity
    Vector6d iir_filtered_cartesian_vel;
    for (int i = 0; i < 6; ++i) {
        double val = cartesian_vel[i];
        for (int j = 0; j < 2; ++j) {
            val = filters_[i][j].process(val);
        }
        iir_filtered_cartesian_vel[i] = val;
    }

    // Apply FIR filtering
    Vector6d filtered_cartesian_vel;
    // for (int i = 0; i < 6; ++i) {
    //     double val = iir_filtered_cartesian_vel[i];
    //     filtered_cartesian_vel[i] = fir_filters_[i].process(val);
    // }

    // Apply SG smoothing filtering
    for (int i = 0; i < 6; ++i){
      double val = iir_filtered_cartesian_vel[i];
      filtered_cartesian_vel[i] = sg_filters_[i].process(val);
    }

    // filtered_cartesian_vel = iir_filtered_cartesian_vel; // DEBUG: Bypass FIR

    filtered_cartesian_vel = state_vector.segment<6>(7); // Use Kalman filtered state for cartesian velocity

    twist_filtered.twist.linear.x  = filtered_cartesian_vel[0];
    twist_filtered.twist.linear.y  = filtered_cartesian_vel[1];
    twist_filtered.twist.linear.z  = filtered_cartesian_vel[2];
    twist_filtered.twist.angular.x = filtered_cartesian_vel[3];
    twist_filtered.twist.angular.y = filtered_cartesian_vel[4];
    twist_filtered.twist.angular.z = filtered_cartesian_vel[5];

    twist.twist.linear.x  = cartesian_vel[0];
    twist.twist.linear.y  = cartesian_vel[1];
    twist.twist.linear.z  = cartesian_vel[2];
    twist.twist.angular.x = cartesian_vel[3];
    twist.twist.angular.y = cartesian_vel[4];
    twist.twist.angular.z = cartesian_vel[5];

    current_cartesian_vel_ = filtered_cartesian_vel;    

    filt_cart_state_pub_.publish(twist_filtered);
    cart_state_pub_.publish(twist);
  }

  // Nodes
  ros::Publisher                            joint_pub_;
  ros::Publisher                            cart_state_pub_;
  ros::Publisher                            filt_cart_state_pub_;
  ros::Publisher                            controller_pub_;
  ros::Publisher                            joint_vel_pub_;
  ros::Subscriber                           joint_state_sub_;
  ros::Subscriber                           cart_vel_sub_;

  // Filters
  double                                    sampling_rate_;
  double                                    cutoff_freq_;
  std::vector<std::vector<Butterworth2nd>>  filters_;
  std::vector<FIRFilter>                    fir_filters_;
  std::vector<SGFilter>                     sg_filters_;
  JointKalmanFilter                         kalman_filter_;

  // Differentiated Velocity
  Vector6d                                  prev_joint_pos_;
  ros::Time                                 prev_joint_time_;
  bool                                      has_prev_joint_state_ = false;

  // Conversion
  Vector6d                                  current_cartesian_vel_;
  Matrix6x6d                                jacobian_;              

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_direct");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  MotionServer server(nh);
  ros::waitForShutdown();
  return 0;
}