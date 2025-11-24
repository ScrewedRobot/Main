

#include <ros/ros.h>
#include <Eigen/Dense>

#include "kin_help.h"
#include "iir_filter.h"
#include "fir_filter.h"

#include <geometry_msgs/TwistStamped.h>
#include <iiwa_msgs/CartesianWrench.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointVelocity.h>
#include "screwed_msgs/DesiredForce.h"


class MotionServer
{
public:
  MotionServer(ros::NodeHandle& nh)
    : kd_filter_(500, 15)
  {

    force_srv_ = nh.advertiseService(
      "force_control",
      &MotionServer::forceControlCB, this);


    // Publishers
    cart_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/iiwa/command/CartesianVelocity", 10);
    // controller_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/iiwa/controller/DesiredVelocity", 10);


    // Subscribers
    wrench_sub_ = nh.subscribe("/iiwa/state/CartesianWrench", 10, &MotionServer::forceCB, this);
    des_force_sub_ = nh.subscribe("/iiwa/command/DesiredForce", 10, &MotionServer::setForceCB, this);

    ROS_INFO("Force Motion Server is ready.");
  }

private:

  // Receives inital force message with controller parameters
  bool forceControlCB(screwed_msgs::DesiredForce::Request& req,
                      screwed_msgs::DesiredForce::Response& res)
  {

    if (req.axis < 1 || req.axis > 3) {
      res.success = false;
      res.message = "Invalid axis specified. Use 1 for x, 2 for y, or 3 for z.";
      return false;
    }

    // Set points
    desired_axis_ = req.axis;
    desired_force_ = req.desired_force + current_force_[desired_axis_ - 1];
    timeout_ = req.timeout;
    kp_ = req.kP;
    ki_ = req.kI;
    kd_ = req.kD;
    force_error_integral_ = 0.0; // Reset integral term
    last_force_time_ = ros::Time::now(); // Reset time
    force_control_start_time_ = ros::Time::now();


    res.success = true;
    res.message = "Force control executed.";
    return true;
  }

  void setForceCB(const iiwa_msgs::CartesianWrench::ConstPtr& msg)
  {
      if (!msg) return;
      if (desired_axis_ == 1) desired_force_ = msg->wrench.force.x;
      else if (desired_axis_ == 2) desired_force_ = msg->wrench.force.y;
      else if (desired_axis_ == 3) desired_force_ = msg->wrench.force.z;
  }

  // Executes force feedback velocity control based on received force messages
  void forceCB(const iiwa_msgs::CartesianWrench::ConstPtr& msg)
  {

    current_force_ = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z};

    if (desired_axis_ == 0) {
      return;
    }

    double measured_force = 0.0;
    if (desired_axis_ == 1) measured_force = msg->wrench.force.x;
    else if (desired_axis_ == 2) measured_force = msg->wrench.force.y;
    else if (desired_axis_ == 3) measured_force = msg->wrench.force.z;

    // Timeout check
    if (timeout_ > 0.0) {
        ros::Duration elapsed = msg->header.stamp - force_control_start_time_;
        if (elapsed.toSec() > timeout_) {
            ROS_WARN("Force control timed out after %.2f seconds.", timeout_);
            desired_axis_ = 0; // disables control
            return;
        }
    }

    double force_error = (desired_force_ - measured_force);

    // Proportional-Integral force controller
    ros::Time now = msg->header.stamp;
    double dt = 0.0;
    if (!last_force_time_.isZero()) {
        dt = (now - last_force_time_).toSec();
    }
    last_force_time_ = now;

    if (dt > 0.0) {
        force_error_integral_ += force_error * dt;
    }

    double force_error_derivative = 0.0;
    if (dt > 0.0) {
        force_error_derivative = (force_error - last_force_error_) / dt;
    }
    last_force_error_ = force_error;

    double filtered_error_derivative = kd_filter_.process(force_error_derivative); // Filter D

    double v_ref = kp_ * force_error + ki_ * force_error_integral_ + kd_ * filtered_error_derivative;

    // Zero out if too small
    if (std::abs(v_ref) < 0.001) v_ref = 0;

    geometry_msgs::TwistStamped control_msg;
    control_msg.header = msg->header;
    control_msg.header.frame_id = "iiwa_link_7";

    Vector6d x_dot = Vector6d::Zero();

    if (desired_axis_ == 1) x_dot[0] = v_ref;
    else if (desired_axis_ == 2) x_dot[1] = v_ref;
    else if (desired_axis_ == 3) x_dot[2] = v_ref;

    control_msg.twist.linear.x  = x_dot[0];
    control_msg.twist.linear.y  = x_dot[1];
    control_msg.twist.linear.z  = x_dot[2];
    
    cart_vel_pub_.publish(control_msg);
    // controller_pub_.publish(control_msg); // Redundant
  }


  // Members
  ros::ServiceServer                        force_srv_;
  ros::Publisher                            cart_vel_pub_;
  ros::Publisher                            controller_pub_;
  ros::Subscriber                           wrench_sub_;
  ros::Subscriber                           des_force_sub_;

  Butterworth2nd                            kd_filter_;
  std::vector<double>                       current_force_;

  int                                       desired_axis_;
  double                                    desired_force_;
  double                                    timeout_;
  double                                    kp_;
  double                                    ki_;
  double                                    kd_;
  double                                    force_error_integral_ = 0.0;
  double                                    last_force_error_;
  ros::Time                                 last_force_time_;
  ros::Time                                 force_control_start_time_;
};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_force");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  MotionServer server(nh);
  ros::waitForShutdown();
  return 0;
}