cmake_minimum_required(VERSION 3.0.2)
project(screwed_planning)

find_package(catkin REQUIRED
  COMPONENTS
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
    screwed_msgs
    message_runtime
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(motion_planning_server src/motion_planning_api.cpp)
target_link_libraries(motion_planning_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_direct 
  src/motion_planning_direct.cpp
  src/kin_help.cpp
  src/iir_filter.cpp
  src/fir_filter.cpp
  src/kalman_filter.cpp)
target_link_libraries(motion_planning_direct ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(motion_planning_force
  src/motion_planning_force.cpp
  src/kin_help.cpp
  src/iir_filter.cpp
  src/fir_filter.cpp)
target_link_libraries(motion_planning_force ${catkin_LIBRARIES} ${Boost_LIBRARIES})


catkin_install_python(PROGRAMS
  scripts/jog_position.py
  scripts/diffik_planner_node.py
  scripts/simple_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)