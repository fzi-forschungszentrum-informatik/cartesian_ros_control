cmake_minimum_required(VERSION 3.0.2)
project(cartesian_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  geometry_msgs
  hardware_interface
  realtime_tools
  roscpp
  )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cartesian_ros_controllers
  CATKIN_DEPENDS
    controller_interface
    geometry_msgs
    hardware_interface
    realtime_tools
    roscpp
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cartesian_ros_controllers
  src/twist_controller.cpp
)
add_dependencies(cartesian_ros_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cartesian_ros_controllers
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark libraries for installation
install(TARGETS cartesian_ros_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(cartesian_state_interface_test test/cartesian_state_interface_test.cpp)
  target_link_libraries(cartesian_state_interface_test ${catkin_LIBRARIES})

  catkin_add_gtest(cartesian_command_interface_test test/cartesian_command_interface_test.cpp)
  target_link_libraries(cartesian_command_interface_test ${catkin_LIBRARIES})
endif()
