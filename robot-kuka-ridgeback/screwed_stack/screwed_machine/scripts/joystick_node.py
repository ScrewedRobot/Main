import rospy
import numpy as np

from sensor_msgs.msg import Joy
from iiwa_msgs.msg import JointPosition, CartesianWrench
from geometry_msgs.msg import TwistStamped

MAX_VEL = 0.05 # m/s
MAX_FORCE = 5 #N

class JoystickNode():


    def __init__(self):
        rospy.init_node('joystick_node', anonymous=False)

        rospy.loginfo("Starting Joy Stick Control Node...")

        # ROS Subscribers
        rospy.Subscriber('bluetooth_teleop/joy', Joy, self.joy_cb, queue_size=1)

        # ROS Publishers
        self.cart_vel_pub = rospy.Publisher("iiwa/command/CartesianVelocity", TwistStamped, queue_size=1)
        self.force_pub = rospy.Publisher("/iiwa/command/DesiredForce", CartesianWrench, queue_size=1)

        self.safety = False
        self.latest_joy = None 
        self.max_vel = MAX_VEL

    def joy_cb(self, msg: Joy):
        self.latest_joy = msg

        last_vel = self.max_vel
        self.max_vel += msg.buttons[13]*(MAX_VEL*0.05)
        self.max_vel -= msg.buttons[14]*(MAX_VEL*0.05)

        if last_vel != self.max_vel:
            rospy.loginfo(f"Max Vel: {self.max_vel}")

        if self.max_vel < 0:
            self.max_vel = 0

    def run(self):
        rate = rospy.Rate(1000) 
        while not rospy.is_shutdown():
            if self.latest_joy is not None:
                msg = self.latest_joy

                # axes mappings
                left_stick_x = msg.axes[0]
                left_stick_y = msg.axes[1]
                left_trig = msg.axes[2]
                right_stick_x = msg.axes[3]
                right_stick_y = msg.axes[4]
                right_trig = msg.axes[5]

                # button mappings
                x_button = msg.buttons[0]
                cir_button = msg.buttons[1]
                sqr_button = msg.buttons[2]
                tri_button = msg.buttons[3]
                left_bump = msg.buttons[4]
                right_bump = msg.buttons[5]
                dpad_up = msg.buttons[13]
                dpad_down = msg.buttons[14]

                # check safety trigger is pressed (and drive safety triggers are not)
                if (left_bump or right_bump) or left_trig > 0:
                    rate.sleep()
                    continue

                # Move EEF in XYZ Cartesian
                xyz_msg = TwistStamped()
                xyz_msg.twist.linear.x = left_stick_x*self.max_vel
                xyz_msg.twist.linear.y = -left_stick_y*self.max_vel
                xyz_msg.twist.linear.z = right_stick_y*self.max_vel

                # Apply Force
                force_msg = CartesianWrench()
                force_msg.wrench.force.z = (right_trig-1)*(MAX_FORCE/2)

                if self.safety:
                    self.force_pub.publish(force_msg)
                    self.cart_vel_pub.publish(xyz_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        task_creator = JoystickNode()
        task_creator.run()
    except rospy.ROSInterruptException:
        pass