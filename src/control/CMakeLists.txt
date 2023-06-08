
cmake_minimum_required(VERSION 2.8.3)
project(control)

# add_library(path_finding path_finding.cpp)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
find_package(vision REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# ## Declare a catkin package
# catkin_package(INCLUDE_DIRS include LIBRARIES path_finding)


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CATKIN_DEVEL_PREFIX}/include src/include)

add_executable(cust_msg src/cust_msg.cpp)
target_link_libraries(cust_msg ${catkin_LIBRARIES})
install(TARGETS cust_msg
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_executable(path_finding src/path_finding.cpp)
# target_link_libraries(path_finding ${catkin_LIBRARIES})
# install(TARGETS path_finding
# 	 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


#add_executable(assignment1 src/assignment1.cpp)
#target_link_libraries(assignment1 ${catkin_LIBRARIES})
#install(TARGETS assignment1
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


add_executable(assignment2 src/assignment2.cpp src/path_finding.cpp)
target_link_libraries(assignment2 ${catkin_LIBRARIES})
install(TARGETS assignment2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# add_executable(assignment3 src/assignment3.cpp)
# target_link_libraries(assignment3 ${catkin_LIBRARIES})
# install(TARGETS assignment3
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
