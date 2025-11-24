#!/usr/bin/env python3

import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def smart_move(group, target_pose, eef_step=0.005):
    waypoints = [group.get_current_pose().pose, target_pose]

    # 1) Try one‑shot Cartesian
    plan, fraction = group.compute_cartesian_path(waypoints, eef_step, True)
    rospy.loginfo("One‑shot Cartesian: %.2f%%", fraction*100)
    if fraction > 0.99:
        return plan

    # 2) Break into finer segments
    rospy.loginfo("Breaking into %d segments", int(1/eef_step))
    start = waypoints[0]
    end   = target_pose
    segments = []
    N = int(1.0 / eef_step)
    for i in range(1, N+1):
        w = copy.deepcopy(start)
        w.position.x += (end.position.x - start.position.x) * i/N
        w.position.y += (end.position.y - start.position.y) * i/N
        w.position.z += (end.position.z - start.position.z) * i/N
        segments.append(w)

    plan, fraction = group.compute_cartesian_path(segments, eef_step/2, True)
    rospy.loginfo("Segmented Cartesian: %.2f%%", fraction*100)
    if fraction > 0.99:
        return plan

    # 3) Global planner fallback
    rospy.logwarn("Cartesian failed, using global planner")
    group.set_pose_target(target_pose)
    plan_full = group.plan()[1]
    return plan_full



def main():
    # 1) Initialize moveit_commander & rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planning_client', anonymous=True)

    # 2) Instantiate RobotCommander, PlanningSceneInterface and MoveGroupCommander
    robot   = moveit_commander.RobotCommander()  
    scene   = moveit_commander.PlanningSceneInterface()
    group   = moveit_commander.MoveGroupCommander("manipulator")

    # 3) Publisher for visualizing plans in Rviz / Foxglove
    display_pub = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=1
    )

    # 4) Give ROS & MoveIt a moment to fill /joint_states, TF, etc.
    rospy.sleep(1.0)

    # 5) Make sure our “start” state is the current state of the robot
    group.set_start_state_to_current_state()

    wpose = group.get_current_pose().pose
    wpose.position.x = 0.29
    wpose.position.y = 0.0
    wpose.position.z = 1.8
 
    plan = smart_move(group, wpose, eef_step=0.01)

    # visualize & execute
    display_msg = moveit_msgs.msg.DisplayTrajectory()
    display_msg.trajectory_start = robot.get_current_state()
    display_msg.trajectory.append(plan)
    display_pub.publish(display_msg)

    sel = input("Execute? (y/n)")
    if sel.lower() == "y":
        print("Executing...")
        group.execute(plan, wait=True)
    else:
        print("Aborting...")

    rospy.loginfo("Demo complete!")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
