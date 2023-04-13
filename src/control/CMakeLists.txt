
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

# add_executable(assignment1 src/assignment1.cpp)
# target_link_libraries(assignment1 ${catkin_LIBRARIES})
# install(TARGETS assignment1
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# add_executable(assignment2 src/assignment2.cpp)
# target_link_libraries(assignment2 ${catkin_LIBRARIES})
# install(TARGETS assignment2
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# add_executable(assignment3 src/assignment3.cpp)
# target_link_libraries(assignment3 ${catkin_LIBRARIES})
# install(TARGETS assignment3
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# add_executable(test src/test.cpp)
# target_link_libraries(test ${catkin_LIBRARIES})
# install(TARGETS test
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

add_executable(ass1D src/assignment1_diffKin.cpp)
target_link_libraries(ass1D ${catkin_LIBRARIES})
install(TARGETS ass1D
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(ass1DT src/difftest.cpp)
target_link_libraries(ass1DT ${catkin_LIBRARIES})
install(TARGETS ass1DT
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
