#!/usr/bin/env python3

import cv2
import rospy
import cv2.aruco as aruco
import numpy as np
import tf.transformations as tft
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped


class ArucoHandler:
    def __init__(self):
        # Parameters
        self.marker_length = 0.08  # meters
        aruco_dict_type = aruco.DICT_5X5_50
        self.aruco_dict = aruco.Dictionary_get(aruco_dict_type)
        self.aruco_params = aruco.DetectorParameters_create()

        # Camera info (must be set via set_camera_matrix)
        self.camera_matrix = None
        self.dist_coeffs = None

        self.bridge = CvBridge()

    def set_camera_matrix(self, camera_matrix, dist_coeffs):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def process_image(self, image_msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logerr("Camera matrix and distortion coefficients must be set before processing images.")
            return [], None, None, None

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return [], cv_image, None, None

        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        transforms = []

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = rvecs[i][0], tvecs[i][0]

            # Skip any invalid translations
            if not np.isfinite(tvec).all() or np.linalg.norm(tvec) > 5.0:
                rospy.logwarn(f"Skipping marker {marker_id} due to invalid tvec")
                continue

            # Build TransformStamped
            t = TransformStamped()
            t.header.stamp = image_msg.header.stamp
            t.header.frame_id = image_msg.header.frame_id
            t.child_frame_id = f"aruco_marker_{marker_id}"

            t.transform.translation.x = float(tvec[0])
            t.transform.translation.y = float(tvec[1])
            t.transform.translation.z = float(tvec[2])

            # Convert rotation vector to matrix
            rot_mat, _ = cv2.Rodrigues(rvec)

            # Camera frame corrections
            cam_correct3 = np.array([[0.0, -1.0, 0.0],
                                     [-1.0, 0.0, 0.0],
                                     [0.0, 0.0, -1.0]])
            cam_correct4 = np.array([[0.0, -1.0, 0.0],
                                     [1.0, 0.0, 0.0],
                                     [0.0, 0.0, 1.0]])

            rot_mat = rot_mat @ cam_correct3 @ cam_correct4

            # Re-orthonormalize rotation matrix (fixes TF_DENORMALIZED_QUATERNION)
            u, _, vh = np.linalg.svd(rot_mat)
            rot_mat = np.dot(u, vh)

            # Convert to quaternion
            quat = self.mat_to_quat(rot_mat)

            # Normalize quaternion to avoid errors
            quat /= np.linalg.norm(quat)

            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            transforms.append(t)

        return transforms, cv_image, ids, None

    def mat_to_quat(self, R):
        """
        Convert a rotation matrix to a quaternion.
        Returns quaternion in [x, y, z, w] format
        """
        q = np.empty((4,))
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2.0
            q[3] = 0.25 * s
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = (R[0, 2] - R[2, 0]) / s
            q[2] = (R[1, 0] - R[0, 1]) / s
        else:
            i = np.argmax(np.diag(R))
            if i == 0:
                s = np.sqrt(R[0, 0] - R[1, 1] - R[2, 2] + 1.0) * 2.0
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif i == 1:
                s = np.sqrt(R[1, 1] - R[0, 0] - R[2, 2] + 1.0) * 2.0
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[1] = 0.25 * s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(R[2, 2] - R[0, 0] - R[1, 1] + 1.0) * 2.0
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[2] = 0.25 * s
                q[0] = 0.0
                q[1] = 0.0
        return q

