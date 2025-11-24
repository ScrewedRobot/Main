// src/motion_planning_server.cpp

#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <screwed_msgs/LimDiffIKinPlanner.h>
#include <screwed_msgs/SimplePlanarPlanner.h>
#include "screwed_msgs/PlanTrajectory.h"
#include "screwed_msgs/TriggerExecute.h"
#include "screwed_msgs/ExecutionQueue.h"
#include "screwed_msgs/CurrentState.h"

class MotionServer
{
public:
  MotionServer(ros::NodeHandle& nh)
    : move_group_("manipulator"),
      is_manual_(false),
      do_replan_(false)
  {
    plan_srv_ = nh.advertiseService(
      "plan_trajectory",
      &MotionServer::planCB, this);

    exec_srv_ = nh.advertiseService(
      "execute_trajectories",
      &MotionServer::executeCB, this);

    queue_pub_ = nh.advertise<screwed_msgs::ExecutionQueue>(
      "queue_size", 1);
    queue_timer_ = nh.createTimer(ros::Duration(0.5), &MotionServer::publishQueue, this);

    diffik_client_ = nh.serviceClient<screwed_msgs::LimDiffIKinPlanner>("diffik_planner");
    simple_client_ = nh.serviceClient<screwed_msgs::SimplePlanarPlanner>("simple_planner");

    jog_cmd_sub_ = nh.subscribe("/state_machine/jog_cmd", 10, &MotionServer::jogCmdCallback, this);
    state_sub_   = nh.subscribe("/state_machine/state",   10, &MotionServer::stateCallback, this);

    ROS_INFO("Ready: /plan_trajectory and /execute_trajectories services up");
  }

private:

  void jogCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!is_manual_) {
      if (do_replan_) {
        // Replan first item in traj_queue_ when switching back to autonomous
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!traj_queue_.empty()) {

            const auto& traj = traj_queue_.front();
            if (!traj.joint_trajectory.points.empty()) {
                move_group_.setStartStateToCurrentState();

                // Build joint map for the goal
                std::map<std::string, double> joint_map;
                const auto& names = traj.joint_trajectory.joint_names;
                const auto& positions = traj.joint_trajectory.points.back().positions;
                for (size_t i = 0; i < names.size(); ++i)
                    joint_map[names[i]] = positions[i];
                move_group_.setJointValueTarget(joint_map);

                // Plan from current state to the goal
                moveit::planning_interface::MoveGroupInterface::Plan new_plan;
                bool ok = (move_group_.plan(new_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (ok) {
                    traj_queue_.front() = new_plan.trajectory_;
                    ROS_INFO("Replanned first trajectory in queue after manual mode.");
                } else {
                    ROS_WARN("Replanning failed for first trajectory in queue after manual mode.");
                }
              }
        }
        do_replan_ = false; 
      }
      return;
    }

    // clear any previous targets
    move_group_.clearPoseTargets();
    move_group_.clearPathConstraints();

    move_group_.setStartStateToCurrentState();
    geometry_msgs::PoseStamped current_pose = move_group_.getCurrentPose();
    geometry_msgs::Pose target_pose = current_pose.pose;

    // handle translation
    target_pose.position.x += msg->linear.x * 0.001;
    target_pose.position.y += msg->linear.y * 0.001;
    target_pose.position.z += msg->linear.z * 0.001;

    // convert degrees to radians
    constexpr double DEG2RAD = M_PI / 180.0;
    double roll  = msg->angular.x * DEG2RAD;
    double pitch = msg->angular.y * DEG2RAD;
    double yaw   = msg->angular.z * DEG2RAD;

    // handle rotation
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::fromMsg(target_pose.orientation, q_orig);
    q_rot.setRPY(roll, pitch, yaw);
    q_new = q_rot * q_orig;
    q_new.normalize();
    target_pose.orientation = tf2::toMsg(q_new);

    // Set the pose target (as in planCB)
    geometry_msgs::Pose target_stamped;
    // target_stamped.header.stamp = ros::Time::now();
    // target_stamped.header.frame_id = move_group_.getPlanningFrame();
    target_stamped = target_pose;
    move_group_.setPoseTarget(target_stamped);

    // Set planning parameters (as in planCB)
    move_group_.allowReplanning(true);
    move_group_.setNumPlanningAttempts(5);
    move_group_.setPlanningTime(5.0);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (ok) {
        move_group_.execute(plan);

        geometry_msgs::PoseStamped new_pose = move_group_.getCurrentPose();
        tf2::Quaternion q;
        tf2::fromMsg(new_pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ROS_INFO_STREAM("\nNew cartesian pose (frame: " << new_pose.header.frame_id << "): \n"
                        << "position (meters): x=" << new_pose.pose.position.x
                        << ", y=" << new_pose.pose.position.y
                        << ", z=" << new_pose.pose.position.z
                        << "\norientation (rpy, deg): "
                        << roll * 180.0 / M_PI << ", "
                        << pitch * 180.0 / M_PI << ", "
                        << yaw * 180.0 / M_PI
                        << "\norientation (quaternion): "
                        << "x=" << new_pose.pose.orientation.x << ", "
                        << "y=" << new_pose.pose.orientation.y << ", "
                        << "z=" << new_pose.pose.orientation.z << ", "
                        << "w=" << new_pose.pose.orientation.w);
    } else {
        ROS_WARN("Jog command: planning failed");
    }
    
  }

  void stateCallback(const screwed_msgs::CurrentState::ConstPtr& msg)
  {
    if (!msg->is_manual) {
      if (is_manual_) {
        is_manual_ = false;
        do_replan_ = true; 
      }
    } else {
      is_manual_ = true;
    }
    
  }

  void publishQueue(const ros::TimerEvent&)
  {
    screwed_msgs::ExecutionQueue msg;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      msg.size = traj_queue_.size();
      msg.goal_poses.clear();

      // Get robot model for FK
      const robot_state::JointModelGroup* jmg = move_group_.getRobotModel()->getJointModelGroup(move_group_.getName());
      robot_state::RobotState state(move_group_.getRobotModel());

      for (const auto& traj : traj_queue_) {
        if (!traj.joint_trajectory.points.empty()) {
          const auto& last_point = traj.joint_trajectory.points.back();
          // Set joint values
          std::map<std::string, double> joint_map;
          for (size_t i = 0; i < traj.joint_trajectory.joint_names.size(); ++i) {
            joint_map[traj.joint_trajectory.joint_names[i]] = last_point.positions[i];
          }
          state.setVariablePositions(joint_map);
          state.update();
          
          geometry_msgs::Pose pose = tf2::toMsg(state.getGlobalLinkTransform(move_group_.getEndEffectorLink()));
          msg.goal_poses.push_back(pose);
        }
      }
    }
    queue_pub_.publish(msg);
  }


  bool planCB(screwed_msgs::PlanTrajectory::Request&  req,
              screwed_msgs::PlanTrajectory::Response& res)
  {

    // clear any previous targets
    move_group_.clearPoseTargets();
    move_group_.clearPathConstraints();

    // pick start state:
    if (traj_queue_.empty())
    {
      move_group_.setStartStateToCurrentState();
    }
    else
    {
      // plan from the end of the last queued trajectory
      const auto& last = traj_queue_.back().joint_trajectory;
      if (!last.points.empty())
      {
        const auto& names     = last.joint_names;
        const auto& positions = last.points.back().positions;

        moveit::core::RobotState start_state(move_group_.getRobotModel());
        start_state.setVariablePositions(names, positions);
        start_state.update();  // compute forward kinematics, etc.
        move_group_.setStartState(start_state);
      }
      else
      {
        move_group_.setStartStateToCurrentState();
      }
    }

    // set the goal
    if (req.use_cartesian)
    {
      // ensure target is stamped in planning frame
      geometry_msgs::Pose target = req.target_pose;
      // target.header.stamp     = ros::Time::now();
      // target.header.frame_id  = move_group_.getPlanningFrame();
      move_group_.setPoseTarget(target);
    }
    else
    {
      // sanity check
      if (req.joint_names.size() != req.joint_positions.size()
          || req.joint_names.empty())
      {
        res.success = false;
        res.message = "joint_names/positions mismatch or empty";
        return true;
      }
      // build name-value map
      std::map<std::string,double> joint_map;
      for (size_t i = 0; i < req.joint_names.size(); ++i)
        joint_map[req.joint_names[i]] = req.joint_positions[i];
      move_group_.setJointValueTarget(joint_map);
    }

    // planning parameters
    move_group_.setPlannerId("RRTConnectkConfigDefault");

    move_group_.allowReplanning(true);
    move_group_.setNumPlanningAttempts(5);
    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(req.max_velocity_scaling_factor);
    move_group_.setMaxAccelerationScalingFactor(req.max_acceleration_scaling_factor);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = false;

    // actually plan
    if (req.use_constraints)
    {

      move_group_.setNumPlanningAttempts(3);
      move_group_.setPlanningTime(5.0);

      std::vector<double> current_joint_values;
      move_group_.getCurrentState()->copyJointGroupPositions(
            move_group_.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_.getName()),
            current_joint_values);

      moveit_msgs::Constraints constraints;
      const std::vector<std::string>& joint_names = move_group_.getVariableNames();

      for (const auto& jc_req : req.constraints.joint_constraints) {
          moveit_msgs::JointConstraint jc;
          jc.joint_name = jc_req.joint_name;

          auto it = std::find(joint_names.begin(), joint_names.end(), jc_req.joint_name);
          if (it != joint_names.end()) {
              size_t idx = std::distance(joint_names.begin(), it);
              jc.position = current_joint_values[idx];
          } else {
              continue;
          }

          jc.tolerance_above = jc_req.tolerance_above;
          jc.tolerance_below = jc_req.tolerance_below;
          jc.weight = jc_req.weight;
          constraints.joint_constraints.push_back(jc);
      }

      move_group_.setPathConstraints(constraints);

      move_group_.setPoseTarget(req.target_pose);

      ok = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    } 
    else
    {
      ok = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }


    // what are the constraints???
    moveit_msgs::Constraints constraints = move_group_.getPathConstraints();
    ROS_INFO_STREAM("Current path constraints: " << constraints);

    // To print joint constraints specifically:
    for (const auto& jc : constraints.joint_constraints) {
        ROS_INFO_STREAM("Joint constraint: " << jc.joint_name
                        << " pos=" << jc.position
                        << " tol_above=" << jc.tolerance_above
                        << " tol_below=" << jc.tolerance_below
                        << " weight=" << jc.weight);
    }



    if (req.use_diffik_planner) 
    {
      screwed_msgs::LimDiffIKinPlanner srv;

      // Handle joint pose
      std::vector<double> joint_values = move_group_.getCurrentJointValues();
      srv.request.start_joint_pose.assign(joint_values.begin(), joint_values.end());
      for (double v : joint_values)
          srv.request.start_joint_pose.push_back(static_cast<float>(v));


      geometry_msgs::Point current_pos = move_group_.getCurrentPose().pose.position;
      geometry_msgs::Point goal_pos = req.target_pose.position;

      // Compute direction vector
      geometry_msgs::Vector3 direction;
      direction.x = goal_pos.x - current_pos.x;
      direction.y = goal_pos.y - current_pos.y;
      direction.z = goal_pos.z - current_pos.z;

      // Normalize
      double norm = std::sqrt(direction.x * direction.x +
                              direction.y * direction.y +
                              direction.z * direction.z);
      if (norm > 1e-6) {
          direction.x /= norm;
          direction.y /= norm;
          direction.z /= norm;
      } else {
          direction.x = direction.y = direction.z = 0.0; // or handle as error
      }

      srv.request.direction.x = direction.x;
      srv.request.direction.y = direction.z;
      srv.request.direction.z = 0;

      ROS_INFO_STREAM("Trajectory Direction: x=" << direction.x << ", y=" << direction.z);
      
      srv.request.distance = static_cast<float>(norm);
      srv.request.speed = req.diffik_speed;
      srv.request.time_step = 0.01; // or from req

      if (diffik_client_.call(srv) && srv.response.success)
      {
        // Convert JointTrajectory to RobotTrajectory
        moveit_msgs::RobotTrajectory robot_traj;
        robot_traj = srv.response.trajectory;

        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          traj_queue_.push_back(robot_traj);
        }
        res.success = true;
        res.message = "DiffIK trajectory queued";
        return true;
      }
      else
      {
        res.success = false;
        res.message = "DiffIK planning failed: " + srv.response.message;
        return true;
      }
    }


    if (req.use_simple_planner) 
    {
      screwed_msgs::SimplePlanarPlanner srv;

      // Handle joint pose
      std::vector<double> joint_values = move_group_.getCurrentJointValues();
      // srv.request.q_start.assign(joint_values.begin(), joint_values.end());
      for (double v : joint_values)
          srv.request.q_start.push_back(static_cast<float>(v));

      geometry_msgs::Point current_pos = move_group_.getCurrentPose().pose.position;
      geometry_msgs::Point goal_pos = req.target_pose.position;

      // Compute displacement 3D
      geometry_msgs::Vector3 displacement;
      displacement.x = goal_pos.x - current_pos.x;
      displacement.y = goal_pos.y - current_pos.y;
      displacement.z = goal_pos.z - current_pos.z;

      // Compute planar displacement and orientation change
      double planar_disp = std::sqrt(displacement.x * displacement.x + displacement.y * displacement.y);
      double dz = displacement.z;

      // Optionally, compute orientation difference if needed (e.g., yaw)
      double dphi = -M_PI/2; // Set this if you want to plan for orientation

      // Fill the service request
      srv.request.xyphi_displacement.x = planar_disp;
      srv.request.xyphi_displacement.y = dz;
      srv.request.xyphi_displacement.z = dphi;

      srv.request.total_time = req.total_time;
      srv.request.eef_steps = 100;

      if (simple_client_.call(srv) && srv.response.success)
      {
        // Convert JointTrajectory to RobotTrajectory
        moveit_msgs::RobotTrajectory robot_traj;
        robot_traj = srv.response.trajectory;

        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          traj_queue_.push_back(robot_traj);
        }
        res.success = true;
        res.message = "Simple trajectory queued";
        return true;
      }
      else
      {
        res.success = false;
        res.message = "Simple planning failed: " + srv.response.message;
        return true;
      }

    }


    // check for success
    if (ok)
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        traj_queue_.push_back(plan.trajectory_);
        res.success = true;
        res.message = "Trajectory queued";
        // visualizeNextTrajectory();
      }
      else
      {
        res.success = false;
        res.message = "Planning failed";
      }
      return true;
    
  }

  bool executeCB(screwed_msgs::TriggerExecute::Request&  req,
                 screwed_msgs::TriggerExecute::Response& res)
  {
    // -1 means “clear the queue”
    if (req.count < 0)
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      traj_queue_.clear();
      res.success = true;
      res.message = "Queue cleared";
      return true;
    }
    if (req.count == 0)
    {
      res.success = false;
      res.message = "Count must be nonzero (use >0 to execute, <0 to clear)";
      return true;
    }

    // execute the next req.count trajectories
    for (int i = 0; i < req.count; ++i)
    {
      moveit_msgs::RobotTrajectory traj;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (traj_queue_.empty())
        {
          res.success = false;
          res.message = "Queue emptied before finishing execution";
          return true;
        }
        traj = std::move(traj_queue_.front());
        traj_queue_.pop_front();
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = std::move(traj);
      bool ok = (move_group_.execute(plan) 
                 == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (!ok)
      {
        res.success = false;
        res.message = "Execution failed at index " + std::to_string(i);
        return true;
      }
    }

    res.success = true;
    res.message = "Executed " + std::to_string(req.count) + " trajectories";
    return true;
  }

  void visualizeNextTrajectory()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (traj_queue_.empty()) {
        ROS_WARN("No trajectory in queue to visualize.");
        return;
    }

    // Prepare DisplayTrajectory message
    moveit_msgs::DisplayTrajectory display_traj;
    // Get current robot state
    moveit_msgs::RobotState current_state;
    robotStateToRobotStateMsg(*move_group_.getCurrentState(), current_state);
    display_traj.trajectory_start = current_state;
    display_traj.trajectory.push_back(traj_queue_.front());

    // Publisher (create once, or make it a member if you want)
    static ros::NodeHandle nh;
    static ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>(
        "/move_group/display_planned_path", 1, true);

    display_pub.publish(display_traj);
    ROS_INFO("Published next trajectory in queue for RViz visualization.");
}

  // Members
  moveit::planning_interface::MoveGroupInterface  move_group_;
  ros::ServiceServer                              plan_srv_, exec_srv_;
  std::deque<moveit_msgs::RobotTrajectory>        traj_queue_;
  std::deque<geometry_msgs::Pose>                 jog_queue_;
  std::mutex                                      queue_mutex_;
  ros::Publisher                                  queue_pub_;
  ros::Timer                                      queue_timer_;
  ros::Subscriber                                 jog_cmd_sub_;
  ros::Subscriber                                 state_sub_;
  ros::ServiceClient                              diffik_client_;
  ros::ServiceClient                              simple_client_;
  bool                                            is_manual_; 
  bool                                            do_replan_; 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_server");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  MotionServer server(nh);
  ros::waitForShutdown();
  return 0;
}
