cmake_minimum_required(VERSION 3.0.2)
project(screwed_machine)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    actionlib
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    tf2_geometry_msgs
    screwed_msgs
    message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


generate_messages(
  DEPENDENCIES
    geometry_msgs
    moveit_msgs
    actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    actionlib
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    tf2_geometry_msgs
    screwed_msgs
    message_runtime
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/state_machine_node.py
  scripts/task_creator_node.py
  scripts/joystick_node.py
  scripts/data_recorder_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)