cmake_minimum_required(VERSION 3.0.2)
project(cartesian_control_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  CartesianPosture.msg
  CartesianTolerance.msg
  CartesianTrajectory.msg
  CartesianTrajectoryPoint.msg
)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_action_files(
  FILES
  FollowCartesianTrajectory.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cartesian_control_msgs
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
#  DEPENDS system_lib
)
