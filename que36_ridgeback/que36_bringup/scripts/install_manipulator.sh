#!/usr/bin/env python3
import os
import robot_upstart

introduction = \
"""Install Manipulator Service:
 - 'manipulator.service' : launch manipulators (drivers and MoveIt!) as background services. 
"""
print(introduction)

interface = os.environ.get('ROBOT_NETWORK')
robot_setup = "/etc/ros/setup.bash"

# manipulator.service
jobname="manipulator"
j = robot_upstart.Job(name=jobname, interface=interface, workspace_setup=robot_setup, master_uri="http://cpr-r100-0150:11311")
j.symlink = True
j.roslaunch_wait = True
j.add(package="que36_bringup", filename="launch/manipulator_bringup.launch")
j.install()

print("Remember to disable manipulator.service: ")
print(" - sudo systemctl disable manipulator")
