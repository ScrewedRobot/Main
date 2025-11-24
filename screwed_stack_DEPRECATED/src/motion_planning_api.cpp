// src/motion_planning_server.cpp

#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/PoseStamped.h>

#include "screwed_stack/PlanTrajectory.h"
#include "screwed_stack/TriggerExecute.h"

class MotionServer
{
public:
  MotionServer(ros::NodeHandle& nh)
    : move_group_("manipulator")
  {
    plan_srv_ = nh.advertiseService(
      "plan_trajectory",
      &MotionServer::planCB, this);

    exec_srv_ = nh.advertiseService(
      "execute_trajectories",
      &MotionServer::executeCB, this);

    ROS_INFO("Ready: /plan_trajectory and /execute_trajectories services up");
  }

private:
  bool planCB(screwed_stack::PlanTrajectory::Request&  req,
              screwed_stack::PlanTrajectory::Response& res)
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
      geometry_msgs::PoseStamped target = req.target_pose;
      target.header.stamp     = ros::Time::now();
      target.header.frame_id  = move_group_.getPlanningFrame();
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
    move_group_.allowReplanning(true);
    move_group_.setNumPlanningAttempts(5);
    move_group_.setPlanningTime(5.0);

    // actually plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_.plan(plan)
               == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (ok)
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      traj_queue_.push_back(plan.trajectory_);
      res.success = true;
      res.message = "Trajectory queued";
    }
    else
    {
      res.success = false;
      res.message = "Planning failed";
    }
    return true;
  }

  bool executeCB(screwed_stack::TriggerExecute::Request&  req,
                 screwed_stack::TriggerExecute::Response& res)
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

  // Members
  moveit::planning_interface::MoveGroupInterface  move_group_;
  ros::ServiceServer                              plan_srv_, exec_srv_;
  std::deque<moveit_msgs::RobotTrajectory>        traj_queue_;
  std::mutex                                      queue_mutex_;
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
