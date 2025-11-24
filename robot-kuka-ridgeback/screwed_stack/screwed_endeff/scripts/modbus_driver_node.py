#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from pymodbus.client import ModbusTcpClient
import time
import sys

IP = "192.168.131.69"
PORT = 502
UNIT_ID = 1
REMOTE_START_ADDR = 4003
PRESET_NUM_ADDR = 4004
RUNTIME = 9  # seconds


class ModbusDriverNode:
    def __init__(self):
        rospy.init_node('modbus_driver_node', anonymous=True)
        rospy.loginfo("Starting Modbus Driver Node...")

        # Connect to Modbus TCP device
        self.client = ModbusTcpClient(IP, port=PORT)
        if self.client.connect():
            rospy.loginfo("Connected to Modbus controller at %s:%d", IP, PORT)
        else:
            rospy.logerr("Failed to connect to Modbus controller")
            rospy.signal_shutdown("Connection failed")
            return

        # Subscribe to trigger topic
        self.trigger_sub = rospy.Subscriber("/screwdriver/trigger", Bool, self.trigger_callback)

        rospy.loginfo("Node ready. Publish to /screwdriver/trigger to start screwdriver.")
        rospy.spin()

    def trigger_callback(self, msg):
        """Callback for /screwdriver/trigger"""
        if msg.data:
            rospy.loginfo("Received trigger ON signal.")
            self.start_screwdriver()
        else:
            rospy.loginfo("Received trigger OFF signal.")
            self.stop_screwdriver()

    def start_screwdriver(self):
        try:
            self.client.write_register(PRESET_NUM_ADDR, 16, unit=UNIT_ID)
            time.sleep(0.2)
            res_on = self.client.write_register(REMOTE_START_ADDR, 1, unit=UNIT_ID)
            rospy.loginfo("Screwdriver started. Response: %s", res_on)

            start_time = time.time()
            while time.time() - start_time < RUNTIME and not rospy.is_shutdown():
                remaining = RUNTIME - int(time.time() - start_time)
                sys.stdout.write(f"\rTime remaining: {remaining:2d} seconds")
                sys.stdout.flush()
                time.sleep(1)

            print()
            self.stop_screwdriver()

        except Exception as e:
            rospy.logerr("Error while starting screwdriver: %s", e)

    def stop_screwdriver(self):
        try:
            res_off = self.client.write_register(REMOTE_START_ADDR, 0, unit=UNIT_ID)
            rospy.loginfo("Screwdriver stopped. Response: %s", res_off)
        except Exception as e:
            rospy.logerr("Error while stopping screwdriver: %s", e)


if __name__ == "__main__":
    try:
        node = ModbusDriverNode()
    except rospy.ROSInterruptException:
        pass
