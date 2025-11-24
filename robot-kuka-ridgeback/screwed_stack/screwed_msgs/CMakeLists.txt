cmake_minimum_required(VERSION 3.0.2)
project(screwed_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
    moveit_msgs
)

# Declare the message files to be built
add_message_files(DIRECTORY msg)

add_service_files(DIRECTORY srv)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES 
    std_msgs 
    geometry_msgs 
    moveit_msgs
)

catkin_package(CATKIN_DEPENDS 
    message_runtime 
    std_msgs 
    geometry_msgs 
    moveit_msgs
)