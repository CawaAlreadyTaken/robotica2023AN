
cmake_minimum_required(VERSION 2.8.3)
project(control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
find_package(vision REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CATKIN_DEVEL_PREFIX}/include)

# add_executable(cust_pub src/cust_pub.cpp)
# target_link_libraries(cust_pub ${catkin_LIBRARIES})
# install(TARGETS cust_pub
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# add_executable(cust_msg src/cust_msg.cpp)
# target_link_libraries(cust_msg ${catkin_LIBRARIES})
# install(TARGETS cust_msg
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

add_executable(assignment1 src/assignment1.cpp)
target_link_libraries(assignment1 ${catkin_LIBRARIES})
install(TARGETS assignment1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)