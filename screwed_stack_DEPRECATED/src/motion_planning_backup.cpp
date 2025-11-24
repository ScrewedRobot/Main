#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/robot_state/conversions.h>             // robotStateToRobotStateMsg
#include <moveit/robot_trajectory/robot_trajectory.h>  // RobotTrajectory
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <geometry_msgs/Pose.h>
#include <vector>

int main(int argc, char** argv)
{
  // ros node setup
  ros::init(argc, argv, "motion_planning_client");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2); 
  spinner.start();

  // setup publisher for showing trajectory
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  move_group.startStateMonitor(1.0);
  ros::Publisher display_pub =
    nh.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path", 1, true);

  move_group.setStartStateToCurrentState();

  // define pose
  // auto current = move_group.getCurrentPose();

  // wait up to 1 second for /joint_states to arrive
  auto current_state = move_group.getCurrentState(1.0);
  // make sure link transforms are up to date
  current_state->updateLinkTransforms();

  // 3) Tell move_group to start from *that* state
  move_group.setStartState(*current_state);

  // grab the end‑effector transform from that state
  const Eigen::Isometry3d &eef_tf =
      current_state->getGlobalLinkTransform(move_group.getEndEffectorLink());
  // pack into a ROS PoseStamped
  geometry_msgs::PoseStamped fresh_pose;
  fresh_pose.header.frame_id = move_group.getPlanningFrame();
  fresh_pose.header.stamp    = ros::Time::now();
  fresh_pose.pose = tf2::toMsg(eef_tf);

  geometry_msgs::Pose target = fresh_pose.pose;
  // target.position.z -= 0.1;  
  target.orientation = fresh_pose.pose.orientation;
  target.position.x = 0.29;
  target.position.y = 0.0;
  target.position.z = 1.8;
  move_group.setPoseTarget(target);


  ROS_INFO("Planning Trajectory...");

  // planning params
  // allow replanning if the first solution fails
  move_group.allowReplanning(true);

  // try more times
  move_group.setNumPlanningAttempts(10);

  // give it up to 20 seconds total
  move_group.setPlanningTime(10.0);

  // plan actual trajectory
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    return 1;

  ROS_INFO("Planning Complete.");

  // publish trajectory
  moveit_msgs::DisplayTrajectory display_msg;

  // convert the current RobotState into a RobotState msg
  moveit_msgs::RobotState start_state_msg;
  moveit::core::robotStateToRobotStateMsg(
      *move_group.getCurrentState(),     // your RobotStatePtr dereferenced
      start_state_msg,                   // output msg
      /*copy_attached_bodies=*/true      // or false if you don’t care about attached objects
  );

  // populate the DisplayTrajectory  
  display_msg.trajectory_start = start_state_msg;  
  display_msg.trajectory.push_back(plan.trajectory_);

  // publish for RViz / Foxglove  
  display_pub.publish(display_msg);

  ROS_INFO("Press Enter to execute…");
  std::cin.get();
  move_group.execute(plan);

  ROS_INFO("Demo complete!");
  ros::shutdown();
  return 0;
}
