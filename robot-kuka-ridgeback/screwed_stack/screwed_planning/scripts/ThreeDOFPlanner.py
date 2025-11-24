#!/usr/bin/env python

import numpy as np
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt


class TrajGen:
    def __init__(self, start_pos, direction, distance, speed, dt):
        self.start = np.array(start_pos, dtype=float)
        self.dir = np.array(direction, dtype=float)
        norm = np.linalg.norm(self.dir)
        assert norm > 0.0, "Direction must be non-zero (x, y > 0)"
        self.dir_norm = self.dir / norm
        self.dist = distance
        self.speed = speed
        self.dt = dt
        self.T = abs(self.dist / self.speed) if self.speed != 0.0 else 0.0
        self.N = int(np.ceil(self.T / self.dt))


    def generate(self):
        times = np.linspace(0, self.N * self.dt, self.N + 1)
        positions = np.tile(self.start, (self.N + 1, 1))
        velocities = np.zeros_like(positions)

        for k, t in enumerate(times):
            traveled = np.sign(self.dist) * min(abs(self.speed * t), abs(self.dist))
            positions[k] = self.start + self.dir_norm * traveled
            if t <= self.T:
                velocities[k] = self.dir_norm * self.speed

        return times, positions, velocities
    
    def plot_planar_trajectory(self):
        times, positions, velocities = self.generate()
        # Only plot X and Y (planar)
        x = positions[:, 0]
        y = positions[:, 1]

        plt.figure()
        plt.plot(x, y, marker='o')
        plt.title("Planar Trajectory")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis('equal')
        plt.grid(True)
        plt.show()


class DiffKinPlanar3R:
    """
    Differential IK for a planar 3R arm with orientation lock on link 3.
    """
    def __init__(self, link_lengths, damping=1e-2):
        self.l1, self.l2, self.l3 = link_lengths
        self.lam = damping

    def forward_kinematics(self, q):
        q1, q2, q3 = q
        q12 = q1 + q2
        q123 = q12 + q3
        x = (self.l1*np.cos(q1) +
             self.l2*np.cos(q12) +
             self.l3*np.cos(q123))
        y = (self.l1*np.sin(q1) +
             self.l2*np.sin(q12) +
             self.l3*np.sin(q123))
        phi3 = q123
        return np.array([x, y, phi3])

    def augmented_jacobian(self, q):
        q1, q2, q3 = q
        q12 = q1 + q2
        q123 = q12 + q3

        J11 = -(self.l1*np.sin(q1) + self.l2*np.sin(q12) + self.l3*np.sin(q123))
        J12 = -(self.l2*np.sin(q12) + self.l3*np.sin(q123))
        J13 = -self.l3*np.sin(q123)
        J21 =  self.l1*np.cos(q1) + self.l2*np.cos(q12) + self.l3*np.cos(q123)
        J22 =  self.l2*np.cos(q12) + self.l3*np.cos(q123)
        J23 =  self.l3*np.cos(q123)

        J_orient = np.array([1.0, 1.0, 1.0])
        J_xy     = np.array([[J11, J12, J13],
                             [J21, J22, J23]])
        return np.vstack((J_xy, J_orient))

    def solve(self, q, v_task):
        """
        Solves for q_dot given:
          q      : (3,) current joint angles
          v_task : (3,) [dot x, dot y, dot phi3(=0)]
        Returns:
          q_dot  : (3,) joint velocities
        """
        J = self.augmented_jacobian(q)
        JJt = J.dot(J.T) + (self.lam**2)*np.eye(3)
        q_dot = J.T.dot(np.linalg.solve(JJt, v_task))
        return q_dot


class DiffIKTrajectoryPlanner3R:
    """
    Combines VectorTrajectoryGenerator and DiffKinPlanar3R to return a RobotTrajectory.
    """
    def __init__(self, link_lengths, joint_names, damping=1e-2):
        """
        Args:
          link_lengths : tuple (l1, l2, l3)
          joint_names  : list of 3 joint names for the planar arm
          damping      : scalar for IK solver
        """
        self.joint_names = joint_names
        self.diffik = DiffKinPlanar3R(link_lengths, damping)

    def plan_along_vector(self, start_q, direction, distance, speed, dt=0.01):
        """
        Compute a RobotTrajectory moving start_q along a given vector.
        
        Args:
          start_q   : list or array of 3 initial joint angles
          direction : array-like (dx, dy, dz) direction for motion
          distance  : total signed displacement along that direction (m)
          speed     : signed speed along that direction (m/s)
          dt        : time step (s)

        Returns:
          traj : moveit_msgs.msg.RobotTrajectory
        """
        # 1) compute start pose in X,Y,phi3
        start_pose = self.diffik.forward_kinematics(start_q)
        # 2) generate Cartesian trajectory along vector
        traj_gen = TrajGen(start_pose, direction, distance, speed, dt)
        times, poses, vels = traj_gen.generate()
        # traj_gen.plot_planar_trajectory()

        # 3) build RobotTrajectory message
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = self.joint_names
        traj.joint_trajectory.points = []

        q = np.array(start_q, dtype=float)
        for k in range(len(times)-1):
            # command velocity in XY plane, phi3=0
            v_task = np.array([vels[k,0], vels[k,1], 0.0])
            q_dot = self.diffik.solve(q, v_task)
            q = q + q_dot * dt

            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            pt.velocities = q_dot.tolist()
            pt.time_from_start = rospy.Duration(times[k+1])
            traj.joint_trajectory.points.append(pt)

        return traj


if __name__ == "__main__":
    rospy.init_node("diffik_vector_test")
    planner = DiffIKTrajectoryPlanner3R(
        link_lengths=(0.4, 0.4, 0.146),
        joint_names=['joint1', 'joint2', 'joint3'],
        damping=1e-2
    )

    traj_msg = planner.plan_along_vector(
        start_q=[1.22, 2.06, -0.96],
        direction=[0, -1, 0],
        distance=0.1,
        speed=0.01,
        dt=0.1
    )
    print(traj_msg)