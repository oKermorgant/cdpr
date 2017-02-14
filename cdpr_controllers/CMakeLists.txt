cmake_minimum_required(VERSION 2.8.3)
project(cdpr_controllers)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cdpr
  roscpp
)

find_package(VISP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cdpr_controllers
  CATKIN_DEPENDS cdpr roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

# basic PID controller for platform control to give an example
add_executable(pid_control src/pid_control.cpp)
target_link_libraries(pid_control ${catkin_LIBRARIES}  ${VISP_LIBRARIES})

# basic PID controller + QP for platform control to give an example
add_executable(qp_pid_control src/qp_pid_control.cpp include/cdpr_controllers/qp.h)
target_link_libraries(qp_pid_control ${catkin_LIBRARIES}  ${VISP_LIBRARIES})
