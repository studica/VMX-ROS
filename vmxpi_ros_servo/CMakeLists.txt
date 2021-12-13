cmake_minimum_required(VERSION 3.0.2)
project(vmxpi_ros_servo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  vmxpi_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vmxpi_ros_servo
#  CATKIN_DEPENDS actionlib roscpp rospy vmxpi_ros_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/vmxpi
  ../vmxpi_ros/include
)


add_library(vmxpi_hal SHARED IMPORTED GLOBAL)
set_target_properties(vmxpi_hal PROPERTIES IMPORTED_LOCATION "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so")
set_target_properties(vmxpi_hal PROPERTIES LINKER_LANGUAGE CXX)

add_library(servo_ros src/Servo_ros.cpp)
target_link_libraries(servo_ros PRIVATE vmxpi_hal ${catkin_LIBRARIES})
install(
  TARGETS servo_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

