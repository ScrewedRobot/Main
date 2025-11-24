#!/usr/bin/env python

import time # only used for sim, use ros time
import yaml
import rospy

from pathlib import Path
from enum import Enum, auto
from geometry_msgs.msg import Pose, PoseStamped
from yaml import load
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


class State(Enum):
        INIT = 0
        STANDBY = 1
        AQUIRE = 2
        APPROACH = 3
        EXECUTE = 4


class StateMachine:

    def __init__(self, ros_logs=True, debugs=True):
        
        self.ros_logs = ros_logs
        self.debugs = debugs

        self.state = State.INIT
        self.prev_state = None

        
        self.configs = self.load_configurations()

        # DO LAST
        self.on_enter_init()


    def run_step(self):

        # Handle state transitions
        if self.state == State.INIT:
            if self.check_standby():
                self.prev_state = self.state
                self.state = self.on_enter_standby()

        elif self.state == State.STANDBY:
            if self.check_aquire():
                self.prev_state = self.state
                self.state = self.on_enter_aquire()

            elif self.check_approach():
                self.prev_state = self.state
                self.state = self.on_enter_approach()

        elif self.state == State.AQUIRE:
            if self.check_standby():
                self.prev_state = self.state
                self.state = self.on_enter_standby()

        elif self.state == State.APPROACH:
            if self.check_standby():
                self.prev_state = self.state
                self.state = self.on_enter_standby()

            elif self.check_execute():
                self.prev_state = self.state
                self.state = self.on_enter_execute()

        elif self.state == State.EXECUTE:
            if self.check_approach():
                self.prev_state = self.state
                self.state = self.on_enter_approach()

    def info_out(self, msg: str):
        if self.ros_logs:
            rospy.loginfo(msg)
        else:
            print("[INFO]: " + str(msg))

    def debug_out(self, msg: str):
        if self.ros_logs:
            rospy.logdebug(msg)
        else:
            if self.debugs:
                print("[DEBUG]: " + str(msg))


    def load_configurations(self):
        this_dir = Path.joinpath(Path(__file__).parent, "../config/saved_configurations.yaml")
        self.debug_out(f"Loading saved configurations from {this_dir}")
        stream = open(this_dir, 'r')
        configs = yaml.load(stream, Loader=Loader)
        self.debug_out(f"Sucessfully loaded configurations: " + str([state.upper() for state in configs]))
        return configs


    # Define Checks
    def check_standby(self) -> bool:
        return False

    def check_aquire(self) -> bool:
        return False

    def check_approach(self) -> bool:
        return False

    def check_execute(self) -> bool:
        return False


    # Define actions when a state is entered
    def on_enter_init(self):
        # Move to HOME
        self.info_out("Moving to HOME")
        self.info_out(self.configs['home']['cartesian_pose']['position'])

        return State.INIT

    def on_enter_standby(self):
        # Move to STANDBY


        return State.STANDBY

    def on_enter_aquire(self):

        return State.AQUIRE

    def on_enter_approach(self):
        # Move to TASK 

        return State.APPROACH

    def on_enter_execute(self):
        

        return State.EXECUTE
    

if __name__ == "__main__":
    # State Machine Simulation
    sm = StateMachine(ros_logs=False, debugs=True)
    for _ in range(5):
        sm.run_step()
        print(f"State: {sm.state.name}")
        time.sleep(0.5)