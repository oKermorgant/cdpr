cmake_minimum_required(VERSION 2.8.3)
project(cdpr)
add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  roslib
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp roslib gazebo_ros sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# TODO: Check names of system library include directories (visp)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# model plugin subscribes to joint efforts and applies them in Gazebo
add_library(cdpr_plugin src/cdpr_plugin.cpp include/cdpr/cdpr_plugin.h)
target_link_libraries(cdpr_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
