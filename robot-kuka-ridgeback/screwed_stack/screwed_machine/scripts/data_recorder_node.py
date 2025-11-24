import rospy
import numpy as np
import csv
import os
import datetime

from iiwa_msgs.msg import JointPositionVelocity, CartesianWrench
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool, SetBoolResponse


TOPIC_PATH = '/iiwa/state/JointPositionVelocity' 

class DataRecorderNode:


    def __init__(self):
        rospy.init_node('data_recorder_node', anonymous=False)

        rospy.loginfo("Starting Data Recorder Node...")

        # ROS Subscribers
        rospy.Subscriber(TOPIC_PATH, JointPositionVelocity, self.recorder_cb, queue_size=1)

        # ROS Service for start/stop recording
        self.recording_service = rospy.Service('toggle_recording', SetBool, self.toggle_recording_cb)

        self.recording = False


    def recorder_cb(self, msg: JointPositionVelocity):
        if not self.recording:
            return

        # Write data to CSV
        timestamp = msg.header.stamp.to_sec()  # Convert to seconds for better readability
        position = [msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5, msg.position.a6, msg.position.a7]
        velocity = [msg.velocity.a1, msg.velocity.a2, msg.velocity.a3, msg.velocity.a4, msg.velocity.a5, msg.velocity.a6, msg.velocity.a7]

        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp] + position + velocity)

    def toggle_recording_cb(self, req):

        if self.recording:
            rospy.loginfo("Stopping recording...")
            rospy.loginfo(f"Data saved to {self.csv_file_path}")
            self.recording = False
        else:
            # Dynamically set the CSV filename with a timestamp
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            self.csv_file_path = os.path.join(os.path.expanduser('~'), f'data_record_{timestamp}.csv')
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                header = ['Timestamp'] + [f'Position_Joint_{i+1}' for i in range(7)] + [f'Velocity_Joint_{i+1}' for i in range(7)]
                writer.writerow(header)
            self.recording = True

        status = "started" if self.recording else "stopped"
        rospy.loginfo(f"Recording {status}.")
        return SetBoolResponse(success=True, message=f"Recording {status}.")


if __name__ == '__main__':
    try:
        task_creator = DataRecorderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass