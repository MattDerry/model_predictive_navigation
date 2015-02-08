cmake_minimum_required(VERSION 2.8.3)
project(model_predictive_navigation)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_msgs
  navfn
  roscpp
  rospy
  std_msgs
  nav_core
  tf
  sensor_msgs
  roslib
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EgoGoal.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include /opt/ros/indigo/include/navfn/
  LIBRARIES control_law
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_msgs navfn roscpp std_msgs nav_core tf sensor_msgs roslib rospy
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /opt/ros/indigo/include/navfn/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(control_law src/control_law.cpp)

## Declare a cpp executable
add_executable(mpepc_trajectory src/mpepc_trajectory.cpp)
add_executable(mpepc_plan src/mpepc_plan.cpp)
add_executable(costmap_translator src/costmap_translator.cpp)
add_executable(costmap_server src/costmap_server.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(mpepc_trajectory ${catkin_EXPORTED_TARGETS})
add_dependencies(mpepc_plan ${catkin_EXPORTED_TARGETS})
add_dependencies(costmap_translator ${catkin_EXPORTED_TARGETS})
add_dependencies(costmap_server ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(mpepc_trajectory
  ${catkin_LIBRARIES}
  control_law
)

target_link_libraries(mpepc_plan
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  control_law
  nlopt
)

target_link_libraries(costmap_translator
  ${catkin_LIBRARIES}
)

target_link_libraries(costmap_server
  ${catkin_LIBRARIES}
)
