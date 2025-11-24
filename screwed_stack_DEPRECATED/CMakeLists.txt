cmake_minimum_required(VERSION 3.0.2)
project(screwed_stack)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib
    rospy
    moveit_commander
    geometry_msgs
    moveit_msgs
    message_generation
  )

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

## Declare service and action files
add_service_files(
  FILES
    PlanTrajectory.srv
    TriggerExecute.srv
)

add_action_files(
  FILES
    ExecuteTraj.action
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    moveit_msgs
    actionlib_msgs
)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    rospy
    actionlib
    actionlib_msgs
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    moveit_commander
    moveit_visual_tools
    rviz_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    geometry_msgs
    moveit_msgs
    message_runtime
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)





add_executable(motion_planning_client src/motion_planning_api.cpp)
target_link_libraries(motion_planning_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_planning_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



#############
## Install ##
#############

## Mark scripts for installation
catkin_install_python(PROGRAMS
  scripts/iiwa_moveit_client.py
  scripts/basic_movement.py
  scripts/jog_position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install service files
install(DIRECTORY srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
  FILES_MATCHING PATTERN "*.srv"
)


## Install action files
install(DIRECTORY action/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action
  FILES_MATCHING PATTERN "*.action"
)


## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

