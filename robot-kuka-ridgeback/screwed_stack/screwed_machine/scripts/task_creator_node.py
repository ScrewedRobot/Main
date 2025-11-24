#!/usr/bin/env python

import yaml
import rospy
import numpy as np
import tf2_ros
import cv2

from aruco_handler.aruco_handler import ArucoHandler
from screwed_msgs.srv import GetTasks, GetTasksResponse
from screwed_msgs.msg import Task, CurrentState
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from pathlib import Path
import tf.transformations as tft
from cv_bridge import CvBridge

class TaskCreator():
    def __init__(self):
        rospy.init_node('task_creator_node', anonymous=True)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # self.publish_static_transform([0, 0, 0.113], [ 0.5, -0.5, -0.5, 0.5 ], "iiwa_link_ee", "reg_marker")
        self.publish_static_transform([0.012362, -0.094006, -0.1227], 
                                      [ 0.1530459, 0.1530459, -0.6903455, 0.6903455 ], 
                                      "kuka_adapter_plate_link", 
                                      "camera_link")
        
        # self.publish_static_transform([0, 0, 0], [ 0, 0, 0, 1 ], "reg_marker", "aruco_marker_10")

        # ROS service servers
        self.get_tasks_srv = rospy.Service('/state_machine/get_tasks', GetTasks, self.get_tasks_callback)

        # ROS Subscribers
        rospy.Subscriber('camera/color/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber('camera/color/image_raw', Image, self.image_callback, queue_size=1)
        rospy.Subscriber('state_machine/state', CurrentState, self.update_tasks, queue_size=1)

        # ROS Publishers
        self.image_pub = rospy.Publisher("aruco_overlay_image", Image, queue_size=1)
        self.bev_pub = rospy.Publisher("bev_overlay", Image, queue_size=1)

        rospy.loginfo("Task Creator Node Initialized")

        self.tasks = self.load_tasks("../config/saved_tasks.yaml")
        self.sm_tasks = []

        self.bev_img = None
        self.tag_center = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.task_frames = dict()

        self.bridge = CvBridge()
        self.aruco_stack = ArucoHandler()

        self.camera_matrix = None
        self.dist_coeffs = None


    def publish_static_transform(self, translation, quaternion, header, child):
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_tf = tf2_ros.TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = header
        static_tf.child_frame_id = child
        static_tf.transform.translation.x = translation[0]
        static_tf.transform.translation.y = translation[1]
        static_tf.transform.translation.z = translation[2]
        static_tf.transform.rotation.x = quaternion[0]
        static_tf.transform.rotation.y = quaternion[1]
        static_tf.transform.rotation.z = quaternion[2]
        static_tf.transform.rotation.w = quaternion[3]
        static_broadcaster.sendTransform(static_tf)
        rospy.loginfo("Published static transform from iiwa_link_ee to reg_marker")

    
    def camera_info_callback(self, camera_info_msg):
        if self.aruco_stack.camera_matrix is not None and self.aruco_stack.dist_coeffs is not None:
            return
        # Extract camera matrix and distortion coefficients
        self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(camera_info_msg.D)

        # Set camera parameters in ArucoHandler
        self.aruco_stack.set_camera_matrix(self.camera_matrix, self.dist_coeffs)
        rospy.loginfo("Camera info received and set in ArucoHandler.")
     

    def update_tasks(self, msg: CurrentState):
        """ 
        Update the task list based on the current state.
        msg: CurrentState message
        """
        self.sm_tasks = msg.task_list
       

    def image_callback(self, image_msg):
        aruco_poses, cv_img, ids, tag_center = self.aruco_stack.process_image(image_msg)
        
        
        if aruco_poses is [] or ids is None:
            self.image_pub.publish(image_msg)
            return
        
        for t, id in zip(aruco_poses, ids):
            self.tf_broadcaster.sendTransform(t)
            # t = self.camera_registration(t)
            # rospy.loginfo(f"Camera to Kuka Adapter: {t.transform}")
            # self.tf_broadcaster.sendTransform(t)

            bev_img = self.birds_eye_view(cv_img, aruco_poses[0])
            self.bev_img = bev_img
            self.tag_center = tag_center
            self.bev_pub.publish(self.bridge.cv2_to_imgmsg(bev_img, "bgr8"))

            if not int(id[0]) in self.task_frames: # Add pose if not in list
                self.task_frames[int(id)] = t.child_frame_id

        if cv_img is not None:
            overlay_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            overlay_msg.header = image_msg.header
            self.image_pub.publish(overlay_msg)



    def get_tasks_callback(self, req):
        ros_tasks = []

        rospy.sleep(6.0)

        # task_layouts = self.load_tasks("../config/task_layouts.yaml")

        # for aruco_id in self.task_frames:
        #     for task_layout in task_layouts:
        #         if int(aruco_id) == int(task_layout['tag_id']):
        #             task_msg = Task()
        #             task_msg.status = int(Task.PENDING)
        #             task_msg.goal_pose = self.get_pose_in_base_link(self.task_frames[aruco_id])
        #             # origin translation
        #             task_msg.goal_pose.position.x += task_layout['origin']['position']['x']
        #             task_msg.goal_pose.position.y += task_layout['origin']['position']['y']
        #             task_msg.goal_pose.position.z += task_layout['origin']['position']['z'] + task_layout['origin']['approach_offset']
        #             task_msg.goal_pose.orientation.x = 0
        #             task_msg.goal_pose.orientation.y = 0
        #             task_msg.goal_pose.orientation.z = 0
        #             task_msg.goal_pose.orientation.w = 1
        #             # task translation
        #             cisbetter = 0
        #             for task in task_layout['targets']:
        #                 target_msg = task_msg
        #                 target_msg.task_id = aruco_id + cisbetter
        #                 target_msg.goal_pose.position.x += task['position']['x']
        #                 target_msg.goal_pose.position.y += task['position']['y']
        #                 target_msg.goal_pose.position.z += task['position']['z']
        #                 quat = self.apply_rot(task_msg.goal_pose.orientation, task['orientation'])
        #                 target_msg.goal_pose.orientation.x = quat[0]
        #                 target_msg.goal_pose.orientation.y = quat[1]
        #                 target_msg.goal_pose.orientation.z = quat[2]
        #                 target_msg.goal_pose.orientation.w = quat[3]
        #                 cisbetter += 1
        #                 ros_tasks.append(target_msg)


        # Return list of acquired poses
        for id in self.task_frames:
            task_msg = Task()
            task_msg.task_id = id
            task_msg.status = int(Task.PENDING)
            task_msg.goal_pose = self.get_pose_in_base_link(self.task_frames[id])
            task_msg.goal_pose.position.x += 0.050
            task_msg.goal_pose.position.y += -0.185
            task_msg.goal_pose.orientation.x = 0.7071068 # -1
            task_msg.goal_pose.orientation.y = 0.7071068 # 0
            task_msg.goal_pose.orientation.z = 0.0 
            task_msg.goal_pose.orientation.w = 0.0
            task_msg.approach_offset = 0.55
            task_msg.fixed_frame = 'base_link'
            ros_tasks.append(task_msg)

            # task_msg = Task()
            # task_msg.task_id = id + 1
            # task_msg.status = int(Task.PENDING)
            # task_msg.goal_pose = self.get_pose_in_base_link(self.task_frames[id])
            # task_msg.goal_pose.position.y -= 0.05
            # task_msg.goal_pose.orientation.x = -1.0
            # task_msg.goal_pose.orientation.y = 0.0
            # task_msg.goal_pose.orientation.z = 0.0
            # task_msg.goal_pose.orientation.w = 0.0
            # task_msg.approach_offset = 0.35
            # task_msg.fixed_frame = 'base_link'
            # ros_tasks.append(task_msg)

            # task_msg = Task()
            # task_msg.task_id = id + 2
            # task_msg.status = int(Task.PENDING)
            # task_msg.goal_pose = self.get_pose_in_base_link(self.task_frames[id])
            # task_msg.goal_pose.position.y -= 0.1
            # task_msg.goal_pose.orientation.x = -1.0
            # task_msg.goal_pose.orientation.y = 0.0
            # task_msg.goal_pose.orientation.z = 0.0
            # task_msg.goal_pose.orientation.w = 0.0
            # task_msg.approach_offset = 0.35
            # task_msg.fixed_frame = 'base_link'
            # ros_tasks.append(task_msg)

            # task_msg = Task()
            # task_msg.task_id = id + 3
            # task_msg.status = int(Task.PENDING)
            # task_msg.goal_pose = self.get_pose_in_base_link(self.task_frames[id])
            # task_msg.goal_pose.position.y -= 0.15
            # task_msg.goal_pose.orientation.x = -1.0
            # task_msg.goal_pose.orientation.y = 0.0
            # task_msg.goal_pose.orientation.z = 0.0
            # task_msg.goal_pose.orientation.w = 0.0
            # task_msg.approach_offset = 0.35
            # task_msg.fixed_frame = 'base_link'
            # ros_tasks.append(task_msg)


        # Put tasks into ros message
        for task in self.tasks:
            task_msg = Task()
            task_msg.task_id = int(task['task_id'])
            task_msg.status = int(Task.PENDING)
            task_msg.goal_pose = self.get_ros_pose(task['goal_pose'])
            task_msg.approach_offset = float(task['parameters']['approach_offset'])
            task_msg.fixed_frame = task['frame']
            # ros_tasks.append(task_msg)

        if ros_tasks == []:
            rospy.logwarn("No tasks found.")
            return GetTasksResponse(success=False, message="No tasks found.")

        # Publish collection
        resp = GetTasksResponse()
        resp.success = True
        resp.message = "Tasks loaded successfully"
        resp.tasks = ros_tasks
        self.sm_tasks = ros_tasks
        return resp

    
    def get_pose_in_base_link(self, source_frame, is_transform=False):

        try:
            # Wait for the transform to be available
            tf2_ros.Duration = rospy.Duration  # for compatibility
            tf = self.tf_buffer.lookup_transform(
                "base_link",
                source_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )

            if is_transform:
                return tf

            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y
            pose.position.z = tf.transform.translation.z
            pose.orientation = tf.transform.rotation

            return pose
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logerr("Transform lookup failed: %s", e)
            return None

    def get_ros_pose(self, pose):
        # Convert pose to ROS Pose message
        ros_pose = Pose()
        ros_pose.position.x = pose['position']['x']
        ros_pose.position.y = pose['position']['y']
        ros_pose.position.z = pose['position']['z']
        ros_pose.orientation.x = pose['orientation']['x']  
        ros_pose.orientation.y = pose['orientation']['y']
        ros_pose.orientation.z = pose['orientation']['z']
        ros_pose.orientation.w = pose['orientation']['w']
        return ros_pose


    def load_tasks(self, local_path):
        this_dir = Path.joinpath(Path(__file__).parent, local_path)
        with open(this_dir, 'r') as file:
            tasks = yaml.safe_load(file)
        return tasks
    

    def apply_rot(self, q_orig, q_rot):
        """
        Apply a rotation to a quaternion.
        q_orig: Original quaternion (x, y, z, w)
        q_rot: Rotation quaternion (x, y, z, w)
        Returns: New quaternion after applying rotation
        """
        q_orig_arr = [q_orig.x, q_orig.y, q_orig.z, q_orig.w]
        q_rot_arr = [q_rot['x'], q_rot['y'], q_rot['z'], q_rot['w']]
        return tft.quaternion_multiply(q_rot_arr, q_orig_arr)
    
    def tf_msg_to_mat(self, tf_msg: TransformStamped):
        """
        Convert a geometry_msgs/TransformStamped into a 4×4 numpy matrix.
        """
        # translation
        t = tf_msg.transform.translation
        # rotation quaternion
        q = tf_msg.transform.rotation
        # build 4×4 from quaternion
        M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        # insert translation
        M[0:3, 3] = [t.x, t.y, t.z]
        return M

    def birds_eye_view(self, cv_image, transform: TransformStamped, height_offset=0.25, center_offset=(0.0, 0.155)):
        if cv_image is None:
            return None
    
        # undistort
        Ks, D = self.camera_matrix, self.dist_coeffs
        h, w = cv_image.shape[:2]
        und = cv2.undistort(cv_image, Ks, D)
        Kv = Ks

        # convert to transformation matrices
        Ts = self.tf_msg_to_mat(transform)
        Tv = np.array([
            [ -1.0,  0.0,  0.0, center_offset[1]],
            [  0.0, -1.0,  0.0, center_offset[0]],
            [  0.0,  0.0,  1.0, -height_offset],
            [  0.0,  0.0,  0.0, 1.0]
        ])

        # compose the homography matrices
        svec = np.array([
            [Ts[0, 0], Ts[0, 1], Ts[0, 3]],
            [Ts[1, 0], Ts[1, 1], Ts[1, 3]],
            [Ts[2, 0], Ts[2, 1], Ts[2, 3]],
        ])
        
        vvec = np.array([
            [Tv[0, 0], Tv[0, 1], Tv[0, 3]],
            [Tv[1, 0], Tv[1, 1], Tv[1, 3]],
            [Tv[2, 0], Tv[2, 1], Tv[2, 3]],
        ])

        Hs = Ks @ svec
        Hv = Kv @ vvec

        H = Hv @ np.linalg.inv(Hs)

        bev_img = cv2.warpPerspective(
            und,
            H,
            (w, h),
            flags=cv2.INTER_LINEAR
        )

        tv = TransformStamped()
        tv.header.stamp = rospy.Time.now()
        tv.header.frame_id = "virtual_camera"
        tv.child_frame_id = "aruco_marker_10"
        tv.transform.translation.x = Tv[0, 3]
        tv.transform.translation.y = Tv[1, 3]
        tv.transform.translation.z = Tv[2, 3]
        quat = tft.quaternion_from_matrix(Tv)
        tv.transform.rotation.x = quat[0]
        tv.transform.rotation.y = quat[1]
        tv.transform.rotation.z = quat[2]
        tv.transform.rotation.w = quat[3]

        # self.tf_broadcaster.sendTransform(tv)

        bev_img = self.overlay_img(bev_img, Hv, (w, h), (0.0, 0.0))

        return bev_img
    

    def overlay_img(self, img, H, size: tuple,  pos: tuple):
        w = size[0]
        h = size[1]

        pt = np.array([pos[0], pos[1], 1.0])
        uvw = H.dot(pt)
        u, v = int(uvw[0]/uvw[2]), int(uvw[1]/uvw[2])

        if 0 <= u < w and 0 <= v < h:
            cv2.circle(img, (u, v), 15, (0, 255, 0), cv2.FILLED)
        
        return img

    def camera_registration(self, ct):
        """
        Multiplies the given 4x4 numpy transformation matrix by the inverse of the
        current transform from 'kuka_adapter_plate_link' to 'iiwa_link_ee'.
        Returns the resulting 4x4 numpy matrix.
        """
        
        try:
            # Wait for the transform to be available
            tf2_ros.Duration = rospy.Duration  # for compatibility
            tf = self.tf_buffer.lookup_transform(
                "kuka_adapter_plate_link",
                "reg_marker",
                rospy.Time(0),
                rospy.Duration(2.0)
            )

            ct2 = self.tf_buffer.lookup_transform(
                "kuka_adapter_plate_link",
                "camera_link",
                rospy.Time(0),
                rospy.Duration(2.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logerr("Transform lookup failed: %s", e)
            return None
        

        print(f"Camera Transform: {ct2.transform}")


        # Convert geometry_msgs/Transform to 4x4 numpy matrix
        t = tf.transform.translation
        q = tf.transform.rotation
        trans = np.array([t.x, t.y, t.z])
        quat = np.array([q.x, q.y, q.z, q.w])

        t2 = ct.transform.translation
        q2 = tf.transform.rotation
        trans2 = np.array([t2.x, t2.y, t2.z])
        quat2 = np.array([q2.x, q2.y, q2.z, q2.w])

        # Build the transformation matrix
        tf_mat = tft.quaternion_matrix(quat)
        tf_mat[0:3, 3] = trans

        ct_mat = tft.quaternion_matrix(quat2)
        ct_mat[0:3, 3] = trans2

        # Invert the transformation
        tf_mat_inv = np.linalg.inv(tf_mat)
        ct_mat_inv = np.linalg.inv(ct_mat)

        # Multiply input matrix by the inverse
        result = np.dot(ct_mat_inv, tf_mat_inv)

        # Convert result (4x4 matrix) to translation and quaternion
        trans = result[0:3, 3]
        quat = tft.quaternion_from_matrix(result)

        tf_msg = tf2_ros.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "kuka_adapter_plate_link"
        tf_msg.child_frame_id = "camera_link"
        tf_msg.transform.translation.x = trans[0]
        tf_msg.transform.translation.y = trans[1]
        tf_msg.transform.translation.z = trans[2]
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        return tf_msg


if __name__ == '__main__':
    try:
        task_creator = TaskCreator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass