import rospy

from ThreeDOFPlanner import DiffIKTrajectoryPlanner3R
from screwed_msgs.srv import LimDiffIKinPlanner, LimDiffIKinPlannerResponse
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy

traj_pub = None

def expand_3dof_to_7dof(traj_3dof, seven_dof_names, fixed_values=None):
    """
    Expand a 3DOF RobotTrajectory to a 7DOF trajectory for KUKA iiwa 7,
    mapping 3DOF to joints 2, 4, 6 and setting joints 1, 3, 5, 7 as redundant.
    Args:
        traj_3dof: moveit_msgs.msg.RobotTrajectory (with 3 joints)
        seven_dof_names: list of 7 joint names (order must match robot)
        fixed_values: list of 4 values for redundant joints (default: zeros)
    Returns:
        moveit_msgs.msg.RobotTrajectory with 7 joints
    """
    if fixed_values is None:
        fixed_values = [0.0, 0.0, 0.0, 0.0]  # for joints 1, 3, 5, 7

    # Indices for 7DOF: [1, 3, 5, 7] are redundant, [2, 4, 6] get 3DOF values
    # seven_dof_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']

    traj_7dof = RobotTrajectory()
    traj_7dof.joint_trajectory.joint_names = seven_dof_names

    for pt in traj_3dof.joint_trajectory.points:
        new_pt = JointTrajectoryPoint()
        # Build the 7DOF positions/velocities
        pos = [0.0]*7
        vel = [0.0]*7
        acc = [0.0]*7 if pt.accelerations else []
        eff = [0.0]*7 if pt.effort else []

        # Set redundant joints (1,3,5,7) to fixed values
        pos[0] = fixed_values[0]
        pos[2] = fixed_values[1]
        pos[4] = fixed_values[2]
        pos[6] = fixed_values[3]

        # Map 3DOF trajectory to joints 2,4,6 (indices 1,3,5)
        pos[1] = pt.positions[0]
        pos[3] = pt.positions[1]
        pos[5] = pt.positions[2]

        # Velocities
        vel[1] = pt.velocities[0]
        vel[3] = pt.velocities[1]
        vel[5] = pt.velocities[2]
        # Redundant joint velocities remain zero

        # Accelerations (if present)
        if pt.accelerations:
            acc[1] = pt.accelerations[0]
            acc[3] = pt.accelerations[1]
            acc[5] = pt.accelerations[2]
        # Effort (if present)
        if pt.effort:
            eff[1] = pt.effort[0]
            eff[3] = pt.effort[1]
            eff[5] = pt.effort[2]

        new_pt.positions = pos
        new_pt.velocities = vel
        if pt.accelerations:
            new_pt.accelerations = acc
        if pt.effort:
            new_pt.effort = eff
        new_pt.time_from_start = deepcopy(pt.time_from_start)
        traj_7dof.joint_trajectory.points.append(new_pt)

    return traj_7dof


def diffik_callback(req):
    rospy.loginfo("Received diffik planning request")

    joint_names = [
        "iiwa_joint_1",
        "iiwa_joint_2",
        "iiwa_joint_3",
        "iiwa_joint_4",
        "iiwa_joint_5",
        "iiwa_joint_6",
        "iiwa_joint_7"
    ]

    link_lengths = (
        0.4, # From Joint 2 to Joint 4
        0.4, # From Joint 4 to Joint 6
        0.146  # From Joint 6 to Joint 7
    )

    planner = DiffIKTrajectoryPlanner3R(
        link_lengths=link_lengths,
        joint_names=[joint_names[1], joint_names[3], joint_names[5]],
        damping=0
    )

    # traj_msg = planner.plan_along_vector(
    #     start_q=[req.start_joint_pose[1], req.start_joint_pose[3], req.start_joint_pose[5]],
    #     direction=[req.direction.x, req.direction.y, req.direction.z],
    #     distance=req.distance,
    #     speed=req.speed,
    #     dt=req.time_step
    # )

    traj_msg = planner.plan_along_vector(
        start_q=[req.start_joint_pose[1], req.start_joint_pose[3], req.start_joint_pose[5]],
        direction=[0, -1, 0],
        distance=0.1,
        speed=req.speed,
        dt=req.time_step
    )

    start_values = [
        req.start_joint_pose[0],
        req.start_joint_pose[2],
        req.start_joint_pose[4],
        req.start_joint_pose[6]
    ]

    real_traj = expand_3dof_to_7dof(traj_msg, joint_names, fixed_values=start_values)

    # --- Publish the trajectory for visualization ---
    if traj_pub is not None:
        traj_pub.publish(real_traj.joint_trajectory)
        rospy.loginfo("Published trajectory to /diffik_trajectory")

    success = True
    message = "DiffIK planning completed."

    return LimDiffIKinPlannerResponse(success=success, message=message, trajectory=real_traj)

def main():
    global traj_pub
    rospy.init_node('diffik_planner_node')
    traj_pub = rospy.Publisher('/diffik_trajectory', JointTrajectory, queue_size=1)
    service = rospy.Service('diffik_planner', LimDiffIKinPlanner, diffik_callback)
    rospy.loginfo("DiffIK Planner Service ready at /diffik_plan")
    rospy.spin()

if __name__ == "__main__":
    main()