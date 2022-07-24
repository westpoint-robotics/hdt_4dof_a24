cmake_minimum_required(VERSION 2.8.3)
project(jointstate_relay_control)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joint_trajectory_controller
  moveit_simple_controller_manager
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES jointstate_relay_control
  CATKIN_DEPENDS  controller_manager hardware_interface joint_trajectory_controller moveit_simple_controller_manager roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_executable(relay_control
  src/relay_control.cpp
  src/relay_interface.cpp
)

target_link_libraries(relay_control
  ${catkin_LIBRARIES}
)

#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
