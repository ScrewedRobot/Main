#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32
from pymodbus.client import ModbusTcpClient
import time
import threading

# Modbus connection parameters
IP = "192.168.131.69"
PORT = 502
UNIT_ID = 1

# Register addresses
REMOTE_START_ADDR = 4003
PRESET_NUM_ADDR = 4004
TORQUE_ADDR = 3300       # Converted torque (*100)
SPEED_ADDR = 3301        # Motor speed
RUN_STATUS_ADDR = 3307   # Bit 0 = motor running
TEMP_ADDR = 3313         # Motor temp, scaled *10

# Preset
PRESET_VALUE = 16  # Multisequence A

AUTO_STOP_SECONDS = 9.5


class ModbusDriverMetricsNode:
    def __init__(self):
        rospy.init_node('modbus_driver_metrics_node', anonymous=True)
        rospy.loginfo("Starting Modbus Driver Metrics Node...")

        # Publishers
        self.torque_pub = rospy.Publisher('/screwdriver/torque_raw', Float32, queue_size=10)
        self.torque_conv_pub = rospy.Publisher('/screwdriver/torque', Float32, queue_size=10)
        self.speed_pub = rospy.Publisher('/screwdriver/speed', Float32, queue_size=10)
        self.duration_pub = rospy.Publisher('/screwdriver/duration', Float32, queue_size=10)
        self.temp_pub = rospy.Publisher('/screwdriver/motor_temp', Float32, queue_size=10)
        self.running_pub = rospy.Publisher('/screwdriver/motor_running', Bool, queue_size=10)

        # Connect to Modbus controller
        self.client = ModbusTcpClient(IP, port=PORT)
        if not self.client.connect():
            rospy.logerr("Failed to connect to Modbus controller at %s:%d", IP, PORT)
            rospy.signal_shutdown("Connection failed")
            return
        rospy.loginfo("Connected to Modbus controller.")

        # Internal state
        self.running = False
        self.start_time = None
        self.auto_stop_timer = None

        # Subscriber to start/stop screwdriver
        rospy.Subscriber("/screwdriver/trigger", Bool, self.trigger_callback)

        rospy.loginfo("Ready. Publish True/False to /screwdriver/trigger to start/stop.")
        self.loop()

    # ------------------------------------------------------
    # Timer-based stop logic
    # ------------------------------------------------------
    def start_auto_stop_timer(self):
        if self.auto_stop_timer:
            self.auto_stop_timer.cancel()

        self.auto_stop_timer = threading.Timer(AUTO_STOP_SECONDS, self.stop_screwdriver)
        self.auto_stop_timer.start()
        rospy.loginfo("Auto-stop timer started (%.1f seconds).", AUTO_STOP_SECONDS)

    # ------------------------------------------------------
    # Start/Stop Logic
    # ------------------------------------------------------
    def trigger_callback(self, msg):
        if msg.data and not self.running:
            self.start_screwdriver()
            self.start_auto_stop_timer()

        elif not msg.data and self.running:
            # Manual stop
            if self.auto_stop_timer:
                self.auto_stop_timer.cancel()
            self.stop_screwdriver()

    def start_screwdriver(self):
        try:
            # Set preset
            self.client.write_register(PRESET_NUM_ADDR, PRESET_VALUE, unit=UNIT_ID)
            time.sleep(0.2)

            # Start screwdriver
            res = self.client.write_register(REMOTE_START_ADDR, 1, unit=UNIT_ID)
            rospy.loginfo("START command sent: %s", res)

            self.running = True
            self.start_time = rospy.Time.now()

        except Exception as e:
            rospy.logerr("Failed to start screwdriver: %s", e)
            self.running = False

    def stop_screwdriver(self):
        try:
            if not self.running:
                return

            res = self.client.write_register(REMOTE_START_ADDR, 0, unit=UNIT_ID)
            rospy.loginfo("STOP command sent: %s", res)

            self.running = False
            self.start_time = None

        except Exception as e:
            rospy.logerr("Failed to stop screwdriver: %s", e)

    # ------------------------------------------------------
    # Main loop
    # ------------------------------------------------------
    def loop(self):
        rate = rospy.Rate(5)  # 5 Hz
        while not rospy.is_shutdown():
            try:
                # Read all needed registers
                torque_raw = self.client.read_input_registers(TORQUE_ADDR, 1, unit=UNIT_ID).registers[0]
                speed = self.client.read_input_registers(SPEED_ADDR, 1, unit=UNIT_ID).registers[0]
                status_reg = self.client.read_input_registers(RUN_STATUS_ADDR, 1, unit=UNIT_ID).registers[0]
                temp_raw = self.client.read_input_registers(TEMP_ADDR, 1, unit=UNIT_ID).registers[0]

                # Process data
                motor_running = bool(status_reg & 0x01)
                torque = torque_raw / 100.0
                motor_temp = temp_raw / 10.0
                duration = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0.0

                # Publish
                self.torque_pub.publish(torque_raw)
                self.torque_conv_pub.publish(torque)
                self.speed_pub.publish(speed)
                self.temp_pub.publish(motor_temp)
                self.running_pub.publish(motor_running)
                self.duration_pub.publish(duration)

            except Exception as e:
                rospy.logwarn_throttle(5, "Modbus read error: %s", e)

            rate.sleep()


if __name__ == "__main__":
    try:
        ModbusDriverMetricsNode()
    except rospy.ROSInterruptException:
        pass

