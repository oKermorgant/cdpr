cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  log2plot
  roscpp
  rospy
  std_msgs
  cdpr
  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES trajectory_generator
  CATKIN_DEPENDS cdpr log2plot roscpp
#  DEPENDS system_lib
)

find_package(VISP REQUIRED)
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

add_executable( trajectory
 	 	src/trajectory.cpp
	 	include/trajectory_generator/trajectory.h	
		)   
target_link_libraries(trajectory ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable( s_curve
 	 	src/s_curve.cpp
	 	include/trajectory_generator/s_curve.h

		)   
target_link_libraries(s_curve ${catkin_LIBRARIES} ${VISP_LIBRARIES})

