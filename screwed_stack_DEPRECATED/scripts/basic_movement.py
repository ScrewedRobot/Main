#!/usr/bin/env python
import sys, rospy
import moveit_commander
import geometry_msgs.msg

def extract_plan(result):
    if isinstance(result, tuple):
        return result[0]
    return result


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('iiwa_moveit_client')

    # give joint_state relay a moment
    rospy.sleep(1.0)

    robot = moveit_commander.RobotCommander()
    print("Groups:", robot.get_group_names())

    group = moveit_commander.MoveGroupCommander("manipulator")

    # use whatever planning frame you have
    frame = group.get_planning_frame()
    group.set_pose_reference_frame(frame)
    group.set_planning_time(10)
    group.set_num_planning_attempts(5)

    group.set_start_state_to_current_state()

    # PART A: Cartesian
    move1 = input("Move 1? (y/n) - Cartesian\n")
    if move1.lower() == 'y':
        # ensure no residual velocity
        group.stop()
        group.clear_pose_targets()

        current = group.get_current_pose().pose
        print("Current Pose: ", current)
        target = geometry_msgs.msg.Pose()
        target.orientation = current.orientation
        target.position.x = current.position.x + 0.1
        target.position.y = current.position.y
        target.position.z = current.position.z - 0.1
        group.set_pose_reference_frame("base_link")

        print("New Pose: ", target)

        (traj_cart, frac_cart) = group.compute_cartesian_path(
            [target],   # waypoints
            0.005,       # eef_step
            False        # avoid_collisions
        )
        if frac_cart < 0.99:
            rospy.logerr("Cartesian only %.1f%%", frac_cart*100)
        else:
            rospy.loginfo("Cartesian OK, executing…")
            group.execute(traj_cart, wait=True)

    # # PART B: Joint‑space
    # input("Enter → Joint‑space…")

    # # ensure no residual velocity
    # group.stop()
    # group.clear_pose_targets()

    # # build your joint_goal however you like; here we just re‑use current:
    # joint_goal = group.get_current_joint_values()
    # # … tweak joint_goal[i] = … if desired …
    # joint_goal[0] += 0.2
    # group.set_joint_value_target(joint_goal)

    # # plan() returns a RobotTrajectory (or False on failure)
    # raw_plan2 = group.plan()
    # plan2 = extract_plan(raw_plan2)

    # if not plan2 or not hasattr(plan2, 'joint_trajectory') \
    #    or len(plan2.joint_trajectory.points) == 0:
    #     rospy.logerr("Joint‑space planning failed (empty trajectory)")
    # else:
    #     rospy.loginfo("Joint‑space plan found, executing…")
    #     group.execute(plan2, wait=True)


    # Go Home
    move2 = input("Move 2? (y/n) - Home\n")
    if move2.lower() == 'y':
        group.stop()
        group.clear_pose_targets()
        group.set_named_target("Straight")
        group.go(wait=True)

        
if __name__ == "__main__":
    main()
