cmake_minimum_required(VERSION 2.8.3)
project(cdpr_controllers)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cdpr
  roscpp
  log2plot
)

find_package(VISP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cdpr_controllers
  CATKIN_DEPENDS cdpr roscpp log2plot
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

add_library(${PROJECT_NAME}
    include/cdpr_controllers/butterworth.h
    include/cdpr_controllers/ctd.h
    src/ctd.cpp                )


# PID controller to try CTD's (Olivier's code)
add_executable(	pid_control 
    src/pid_control.cpp)
target_link_libraries(pid_control ${catkin_LIBRARIES}  ${VISP_LIBRARIES} ${PROJECT_NAME})

# basic PID controller + QP for platform control to give an example
add_executable(	qp_pid_control 
	       	src/qp_pid_control.cpp 
		include/cdpr_controllers/qp.h)
target_link_libraries(qp_pid_control ${catkin_LIBRARIES}  ${VISP_LIBRARIES})

# Feekback linearization controller(CTC) in Cartesian space for platform control to give an example

add_executable( CTC_qp_taskspace
 	 	src/CTC_qp_taskspace.cpp
	 	include/cdpr_controllers/qp.h	
		)   
target_link_libraries( CTC_qp_taskspace ${catkin_LIBRARIES} ${VISP_LIBRARIES})

# Feekback linearization controller(CTC) in joint space for platform control to give an example

add_executable( CTC_qp_jointspace
 	 	src/CTC_qp_jointspace.cpp
	 	include/cdpr_controllers/qp.h	
		)   
target_link_libraries( CTC_qp_jointspace ${catkin_LIBRARIES} ${VISP_LIBRARIES})
 
# CTC + Closed form for platform control to give an example
 add_executable(closed_form
 	 	src/closed_form.cpp
	 	include/cdpr_controllers/qp.h	
		)   
target_link_libraries(closed_form ${catkin_LIBRARIES} ${VISP_LIBRARIES})

# CTC + improved QP for platform control to give an example

#add_executable( CTC_qpN_js
#    src/CTC_qpN_js.cpp
#   include/cdpr_controllers/qp.h 
#   )   
#target_link_libraries( CTC_qpN_js ${catkin_LIBRARIES} ${VISP_LIBRARIES})
