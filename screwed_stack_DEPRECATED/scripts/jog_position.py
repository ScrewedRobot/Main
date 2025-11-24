#!/usr/bin/env python
import sys, rospy
import moveit_commander
import geometry_msgs.msg
import math

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion (x, y, z, w).
    
    Arguments:
    - roll: Rotation around X axis in radians
    - pitch: Rotation around Y axis in radians
    - yaw: Rotation around Z axis in radians
    
    Returns:
    (x, y, z, w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return x, y, z, w

def quaternion_to_euler(x: float, y: float, z: float, w: float):
    """
    Convert a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
    
    Arguments:
    - x, y, z, w: Quaternion components
    
    Returns:
    (roll, pitch, yaw) in radians
    """
    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

def extract_plan(result):
    """
    Turn whatever group.plan() returned into either
      - a moveit_msgs/RobotTrajectory, or
      - None if planning failed.
    """
    # 1) outright failure
    if result is False:
        return None

    # 2) a tuple of anything → trajectory is always first
    if isinstance(result, (tuple, list)):
        return result[0]

    # 3) already a RobotTrajectory
    return result

def menu():
    print("Options: ")
    print("1. Jog - Cartesian")
    print("2. Print - Cartesian")
    print("3. Jog - Joint")
    print("4. Print - Joint")
    print("5. Move to State")
    print("6. Exit")
    return input(">>> ")

def cartesian_menu():
    print("Jog - Cartesian")
    print("1. Jog X")
    print("2. Jog Y")
    print("3. Jog Z")
    print("4. Jog Roll")
    print("5. Jog Pitch")
    print("6. Jog Yaw")
    print("7. Back")
    return input(">>> ")

def updated_cartesian_pose(current_pose, axis, increment: float):
    target_pose = geometry_msgs.msg.Pose()
    target_pose = current_pose

    if axis == "X":
        target_pose.position.x = current_pose.position.x + increment
    elif axis == "Y":
        target_pose.position.y = current_pose.position.y + increment
    elif axis == "Z":
        target_pose.position.z = current_pose.position.z + increment

    if axis in ["Roll", "Pitch", "Yaw"]:
        roll, pitch, yaw = quaternion_to_euler(target_pose.orientation.x, 
                                        target_pose.orientation.y, 
                                        target_pose.orientation.z, 
                                        target_pose.orientation.w)
        if axis == "Roll":
            roll += increment
        elif axis == "Pitch":
            ptich += increment
        elif axis == "Yaw":
            yaw += increment

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw

    return target_pose
    
def joint_menu():
    print("Jog - Joint")
    print("1. Jog Joint 1 (Base)")
    print("2. Jog Joint 2")
    print("3. Jog Joint 3")
    print("4. Jog Joint 4")
    print("5. Jog Joint 5")
    print("6. Jog Joint 6")
    print("7. Jog Joint 7 (End-Effector)")
    print("8. Back")
    return input(">>> ")

def state_menu():
    print("Move to State: ")
    print("1. Home") # Stright
    print("2. Standby") # Ready
    print("3. Back")
    return input(">>> ")


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('jog_position')

    group = moveit_commander.MoveGroupCommander("manipulator")

    # use whatever planning frame you have
    frame = group.get_planning_frame()
    group.set_pose_reference_frame(frame)
    group.set_planning_time(10)
    group.set_num_planning_attempts(5)

    selector = 0

    while True:
        # Open user selection menu
        selector = int(menu())
        if selector == 6:
            break

        # Get the current pose
        current_pose = group.get_current_pose().pose

        # Get the current joint values
        joint_values = group.get_current_joint_values()

        # 1. Jog - Cartesian
        if selector == 1:
            cart_sel = int(cartesian_menu())
            pose = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
            incr = float(input(f"Jog {pose[cart_sel-1]} by (meters): "))

            group.shift_pose_target(axis=cart_sel-1, value=incr)

            success = group.go(wait=True)
            if not success:
                rospy.logerr("Motion planning/execution failed")

            # -------------------------------------
            # raw = group.plan()

            # # 1) check for outright failure
            # if raw is False:
            #     rospy.logerr("Planning failed outright (got False)")
            #     return

            # # 2) extract the trajectory from whatever raw is
            # traj = raw[0] if isinstance(raw, (tuple, list)) else raw

            # # 3) make sure it’s non‑empty
            # if not hasattr(traj, 'joint_trajectory') or not traj.joint_trajectory.points:
            #     rospy.logerr("Plan succeeded but trajectory is empty")
            #     return

            # # 4) execute the valid trajectory
            # group.execute(traj, wait=True)
            # -------------------------------------------

            # goal_pose = updated_cartesian_pose(current_pose, pose[cart_sel-1], incr)

            # (traj_cart, frac_cart) = group.compute_cartesian_path(
            # [goal_pose],   # waypoints
            # 0.005,       # eef_step
            # False        # avoid_collisions
            # )

            # if frac_cart < 0.99:
            #     rospy.logerr("Cartesian only %.1f%%", frac_cart*100)
            # else:
            #     rospy.loginfo("Cartesian OK, executing…")
            #     group.execute(traj_cart, wait=True)

        # 2. Print - Cartesian
        elif selector == 2:
            print("Current Cartesian Pose: ")
            print(current_pose)

        # 3. Jog - Joint
        elif selector == 3:
            joint_sel = int(joint_menu())
            j_incr = float(input(f"Jog Joint {joint_sel} by (deg): "))
            joint_values[joint_sel-1] += j_incr*(math.pi/180)
            group.set_joint_value_target(joint_values)
            group.go()

        # 4. Print - Joint
        elif selector == 4:
            print("Current Joint Values: ")
            print(joint_values)

        # 5. Move to State
        elif selector == 5:
            state_sel = int(state_menu())
            group.clear_pose_targets()

            if state_sel == 1:
                group.set_named_target("Straight")
            elif state_sel == 2:
                group.set_named_target("Ready")
            
            if state_sel != 3:      
                group.go(wait=True)


        # ensure no residual velocity
        group.stop()
        group.clear_pose_targets()

        
if __name__ == "__main__":
    main()
