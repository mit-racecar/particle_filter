cmake_minimum_required(VERSION 3.5)
project(particle_filter)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

# Specify catkin package
catkin_package(
  CATKIN_DEPENDS  message_runtime rospy sensor_msgs std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Install python nodes
install(PROGRAMS 
src/particle_filter.py 
src/utils.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
        launch
        maps
        rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
