#!/usr/bin/env python

import rospy
import numpy as np

from screwed_msgs.srv import SimplePlanarPlanner, SimplePlanarPlannerResponse, SimplePlanarPlannerRequest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import matplotlib.pyplot as plt
import matplotlib.animation as animation


def animate_trajectory(manipulator, q_traj, interval=50):
    """
    Animate the manipulator following the given joint trajectory.
    manipulator: PlanarManipulator instance
    q_traj: list of [q1, q2, q3] joint configurations
    interval: delay between frames in ms
    """
    fig, ax = plt.subplots()
    ax.set_title("Planar Manipulator Animation")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis('equal')
    ax.grid(True)

    # q_traj = q_traj[::-1]

    # Compute workspace bounds for better view
    all_points = []
    for q in q_traj:
        pts = manipulator.forward_kin(q)
        all_points.append(pts)
    all_points = np.array(all_points)
    ax.set_xlim(np.min(all_points[:,:,0])-0.25, np.max(all_points[:,:,0])+0.25)
    ax.set_ylim(np.min(all_points[:,:,1])-0.25, np.max(all_points[:,:,1])+0.25)

    line, = ax.plot([], [], 'o-', lw=2, label='Manipulator')
    ax.legend()

    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        pts = manipulator.forward_kin(q_traj[frame])
        line.set_data(pts[:,0], pts[:,1])
        return line,

    ani = animation.FuncAnimation(
        fig, update, frames=len(q_traj), init_func=init,
        blit=True, interval=interval, repeat=False
    )
    plt.show()


class PlanarManipulator:

    def __init__(self, link_lengths):
        self.link_lengths = np.array(link_lengths)

    # def forward_kin(self, joint_angles):
    #     l1, l2, l3 = self.link_lengths
    #     q1, q2, q3 = joint_angles

        # c1, s1 = np.cos(q1), np.sin(q1)
        # c2, s2 = np.cos(q2), np.sin(q2)
        # c3, s3 = np.cos(q3), np.sin(q3)
        
        # A00 = np.array([
        #     [ 1,   0, 0,     0],
        #     [ 0,  -1, 0,     0],
        #     [ 0,   0, 1,     0],
        #     [ 0,   0, 0,     1]
        # ])

        # A01 = np.array([
        #     [c1, -s1, 0, l1*c1],
        #     [s1,  c1, 0, l1*s1],
        #     [ 0,   0, 1,     0],
        #     [ 0,   0, 0,     1]
        # ])

        # A12 = np.array([
        #     [c2, -s2,  0, l2*c2],
        #     [s2,  c2,  0, l2*s2],
        #     [ 0,   0,  1,     0],
        #     [ 0,   0,  0,     1]
        # ])

        # A23 = np.array([
        #     [c3, -s3,  0, l3*c3],
        #     [s3,  c3,  0, l3*s3],
        #     [ 0,   0,  1,     0],
        #     [ 0,   0,  0,     1]
        # ])

        # T01 = A00 @ A01
        # T02 = A00 @ A01 @ A12
        # T03 = A00 @ A01 @ A12 @ A23

        # origin = np.array([0, 0, 0, 1])
        # p1 = T01 @ origin
        # p2 = T02 @ origin
        # p3 = T03 @ origin
        # x0, y0 = 0, 0
        # x1, y1 = p1[0], p1[1]
        # x2, y2 = p2[0], p2[1]
        # x3, y3 = p3[0], p3[1]

        # # origin at 0, 0 (frame: iiwa_link_1)
        # q1 += np.pi/2
        # q2 = -q2

        # x0, y0, = 0, 0
        # x1 = l1 * np.cos(q1)
        # y1 = l1 * np.sin(q1)
        # x2 = x1 + l2 * np.cos(q1 + q2)
        # y2 = y1 + l2 * np.sin(q1 + q2)
        # x3 = x2 + l3 * np.cos(q1 + q2 + q3)
        # y3 = y2 + l3 * np.sin(q1 + q2 + q3)

        # return np.array([[x0, y0], [x1, y1], [x2, y2], [x3, y3]])
    
    def forward_kin(self, joint_angles):
        l1, l2, l3 = self.link_lengths
        q1, q2, q3     = joint_angles

        # no +π/2, no q2 = -q2 in here
        x1 = l1*np.cos(q1)
        y1 = l1*np.sin(q1)

        x2 = x1 + l2*np.cos(q1 + q2)
        y2 = y1 + l2*np.sin(q1 + q2)

        x3 = x2 + l3*np.cos(q1 + q2 + q3)
        y3 = y2 + l3*np.sin(q1 + q2 + q3)

        return np.array([
        [0,   0],
        [x1,  y1],
        [x2,  y2],
        [x3,  y3],
        ])
    

    def jacobian(self, joint_angles):
        l1, l2, l3 = self.link_lengths
        t1, t2, t3 = joint_angles
        a1 = t1
        a2 = t1 + t2
        a3 = t1 + t2 + t3
        j = np.zeros((2, 3))
        # dx/dth
        j[0, 0] = -l1*np.sin(a1) - l2*np.sin(a2) - l3*np.sin(a3)
        j[0, 1] = -l2*np.sin(a2) - l3*np.sin(a3)
        j[0, 2] = -l3*np.sin(a3)
        # dy/dth
        j[1, 0] =  l1*np.cos(a1) + l2*np.cos(a2) + l3*np.cos(a3)
        j[1, 1] =  l2*np.cos(a2) + l3*np.cos(a3)
        j[1, 2] =  l3*np.cos(a3)
        return j
    
    def jacobian_full(self, joint_angles):
        J_pos = self.jacobian(joint_angles)  # (2,3)
        J_phi = np.ones((1, 3))              # dphi/dq = [1, 1, 1]
        return np.vstack((J_pos, J_phi))     # (3,3)

    
    def inverse_kin(self, x_des, y_des, phi, elbow_up=True):
        l1,l2,l3 = self.link_lengths

        # wrist center in the SAME pure frame as forward_kin
        x_w = x_des - l3*np.cos(phi)
        y_w = y_des - l3*np.sin(phi)

        r = np.hypot(x_w, y_w)
        D = (r*r - l1*l1 - l2*l2) / (2*l1*l2)
        # clamp to [-1,1] for safety
        D = np.clip(D, -1.0, +1.0)
        delta = np.sqrt(1 - D*D)

        if elbow_up:
            q2 = np.arctan2(-delta, D)
        else:
            q2 = np.arctan2(delta, D)

        q1 = np.arctan2(y_w, x_w) \
            - np.arctan2(l2*np.sin(q2), l1 + l2*np.cos(q2))

        # **no extra π** here:
        q3 = phi - (q1 + q2)

        return np.array([q1, q2, q3])

    
class PlanningNode:

    def __init__(self):

        self.manipulator = PlanarManipulator([0.4, 0.4, 0.146])

        # ROS service servers
        self.server = rospy.Service('/simple_planner', SimplePlanarPlanner, self.planner_cb)

    def planner_cb(self, req: SimplePlanarPlannerRequest):

        q_kuka = req.q_start
        xy_disp = req.xyphi_displacement

        # print(f"Displacement: {xy_disp}")

        # verify that the kuka is in correct configuration
        tol = 1e-2
        if abs(q_kuka[2]) < tol and abs(q_kuka[4]) < tol:
            rospy.loginfo("Robot is in a correct configuration, planning...")
        else:
            rospy.logwarn(f"Robot is NOT in a correct configuration. J3: {q_kuka[2]}, J5: {q_kuka[4]}")
            return SimplePlanarPlannerResponse(success=False)
        
        q_start, q_cached = self.map_dof_7to3(q_kuka)

        # q_traj = self.plan_cartesian(q_start, [xy_disp.x, xy_disp.y], phi=xy_disp.z, steps=req.eef_steps)
        q_traj = self.plan_cartesian(q_start, [xy_disp.x, xy_disp.y], steps=600)

        # animate_trajectory(self.manipulator, q_traj)
        # print(q_traj[0])

        # xy_start = self.manipulator.forward_kin(q_traj[0])
        # print(xy_start)
        # self.plot_joints(xy_start)

        dt = req.total_time / (len(q_traj))

        robot_traj = self.q_to_robot_traj(q_traj, q_cached, dt)

        return SimplePlanarPlannerResponse(success=True, trajectory=robot_traj)


    def plan_cartesian(self, q_start, xy_disp, phi=-np.pi/2, steps=100):
        xy_poses = self.manipulator.forward_kin(q_start)
        xy_start = xy_poses[3]
        # self.plot_joints(xy_poses)
        # print(q_start)
        # print(xy_poses)

        path = np.linspace(xy_start, xy_start + xy_disp, steps)
        q_traj = [q_start]

        for (x, y) in path:
            # print(f"X: {x}, Y: {y}")
            q_curr = self.manipulator.inverse_kin(x, y, phi, elbow_up=True)
            q_traj.append(q_curr)

        return q_traj
    

    def q_to_robot_traj(self, q_traj, q_cached, dt, joint_names=None):
        if joint_names is None:
            joint_names = [
                "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4",
                "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"
            ]

        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = joint_names

        time_from_start = 0.0
        for q in q_traj:
            q7 = self.map_dof_3to7(q, q_cached)
            pt = JointTrajectoryPoint()
            pt.positions = q7.tolist()
            pt.time_from_start = rospy.Duration(time_from_start)
            traj.joint_trajectory.points.append(pt)
            time_from_start += dt

        return traj
    

    def plot_joints(self, xy_poses):
        plt.figure()
        plt.plot(xy_poses[:, 0], xy_poses[:, 1], 'o-', label='Joint positions')
        plt.title("Planar Manipulator Joint Positions (FK)")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()


    def map_dof_3to7(self, q_joints, q_cached):
        
        q_kuka = np.zeros(7)

        q_kuka[1] = q_joints[0] - np.pi/2
        q_kuka[3] = -q_joints[1]
        q_kuka[5] = q_joints[2]

        q_kuka[0] = q_cached[0]
        q_kuka[2] = q_cached[1]
        q_kuka[4] = q_cached[2]
        q_kuka[6] = q_cached[3]

        return q_kuka


    def map_dof_7to3(self, q_kuka):

        q_joints = np.zeros(3)
        q_cached = np.zeros(4)

        q_joints[0] = q_kuka[1] + np.pi/2
        q_joints[1] = -q_kuka[3]
        q_joints[2] = q_kuka[5]

        q_cached[0] = q_kuka[0]
        q_cached[1] = q_kuka[2]
        q_cached[2] = q_kuka[4]
        q_cached[3] = q_kuka[6]

        return q_joints, q_cached


def test_planning():
    # Create PlanningNode instance
    planner = PlanningNode()

    # Example 7-DOF start configuration (all zeros)
    q_start = np.zeros(7)
    q_start[1] = np.pi/4
    q_start[3] = -np.pi/2
    q_start[5] = -np.pi/4

    # Example displacement (move 0.1m in x, 0.0m in y, keep phi at -pi/2)
    class DummyDisp:
        x = 0.0
        y = -0.1
        z = -np.pi/2
    xy_disp = DummyDisp()

    # Example request
    class DummyReq:
        def __init__(self, q_start, xy_disp):
            self.q_start = q_start
            self.xyphi_displacement = xy_disp
            self.eef_steps = 100
            self.total_time = 2.0

    req = DummyReq(q_start, xy_disp)
    # Call the planner callback
    resp = planner.planner_cb(req)

    print("Success:", resp.success)
    print("Trajectory points:")
    ee_path = []
    ee_phi = []
    for pt in resp.trajectory.joint_trajectory.points:
        print("  positions:", np.round(pt.positions, 4), "time:", pt.time_from_start.to_sec())
        q7 = np.array(pt.positions)
        q3 = np.array([q7[1], q7[3], q7[5]])
        fk = planner.manipulator.forward_kin(q3)
        xy = fk[-1]  # end-effector position
        phi = np.sum(q3)  # end-effector angle
        ee_path.append(xy)
        ee_phi.append(phi)

    ee_path = np.array(ee_path)
    ee_phi = np.unwrap(np.array(ee_phi))  # unwrap for smooth angle plot

    # Plot XY path and end-effector angle
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 8))

    ax1.plot(ee_path[:, 0], ee_path[:, 1], marker='o')
    ax1.set_title("End-Effector Planar Path")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.axis('equal')
    ax1.grid(True)

    ax2.plot(ee_phi, marker='.')
    ax2.set_title("End-Effector Angle (phi)")
    ax2.set_xlabel("Trajectory Step")
    ax2.set_ylabel("Angle (rad)")
    ax2.grid(True)

    plt.tight_layout()
    plt.show()
    

if __name__ == "__main__":
    # Uncomment ONE of the following blocks:

    # --- For ROS service node ---
    rospy.init_node('simple_planner_node', anonymous=True)
    node = PlanningNode()
    rospy.loginfo("Simple planner service ready at /state_machine/simple_planner")
    rospy.spin()

    # --- For standalone test (not as a ROS node) ---
    # test_planning()
    