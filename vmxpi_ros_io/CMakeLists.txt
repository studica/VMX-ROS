cmake_minimum_required(VERSION 3.0.2)
project(vmxpi_ros_io)

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
#  LIBRARIES vmxpi_ros_io
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

add_library(vmxpi_hal STATIC IMPORTED GLOBAL)
set_target_properties(vmxpi_hal PROPERTIES IMPORTED_LOCATION "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so")

add_library(digitalin_ros src/DI_ros.cpp)
target_link_libraries(digitalin_ros PRIVATE vmxpi_hal ${catkin_LIBRARIES})
install(
  TARGETS digitalin_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(digitalout_ros src/DO_ros.cpp)
target_link_libraries(digitalout_ros PRIVATE vmxpi_hal ${catkin_LIBRARIES})
install(
  TARGETS digitalout_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(iowd_ros src/IOwd_ros.cpp)
target_link_libraries(iowd_ros PRIVATE vmxpi_hal ${catkin_LIBRARIES})
install(
  TARGETS iowd_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
