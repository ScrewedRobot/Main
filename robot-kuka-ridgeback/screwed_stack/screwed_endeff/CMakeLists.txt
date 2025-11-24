cmake_minimum_required(VERSION 3.0.2)
project(screwed_endeff)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    screwed_msgs
    message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
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

add_executable(endeff_node src/endeff_node.cpp)
target_link_libraries(endeff_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)