
cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
   rospy
   message_generation
   std_msgs
)

add_message_files(
   FILES
   custMsg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)


## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)



## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})