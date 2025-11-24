# que36_ridgeback
## Ridgeback + Kuka Manipulator LBR iiwa 7 R800 + Ewellix Lift Column + SICK LMS-111 + UM7 IMU

#Create Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src --rosdistro noetic -y -r
catkin_make

#Grab Dependencies (Lift + Kuka Drivers)
wstool init src
wstool merge -t src ~/catkin_ws/src/que36_ridgeback/que36_ridgeback.rosinstall
wstool update -t src

#UM7 IMU
sudo apt-get install ros-$ROS_DISTRO-um7

#UM7 UDEV
https://gitlab.clearpathrobotics.com/research/clearpath_iso/-/blob/master/clearpath/files/41-clearpath.rules

#UM7 ENVARS

#UM7 Launch
roslaunch que36_bringup um7.launch


#EWELLIX LIFT COLUMN
https://wiki.clearpathrobotics.com/display/ridge/Ewellix+Lifting+Column
https://medialibrary.ewellix.com/asset/16981

cd ~/catkin_ws/src
git clone https://github.com/clearpathrobotics/ewellix_tlt.git
sudo apt-get install -y libserial-dev
sudo apt-get install ros-noetic-serial
cd ..
catkin_make
