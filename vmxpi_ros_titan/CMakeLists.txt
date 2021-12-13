cmake_minimum_required(VERSION 3.0.2)
project(vmxpi_ros_titan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  vmxpi_ros
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

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
  # INCLUDE_DIRS include
  # LIBRARIES vmxpi_ros
  # CATKIN_DEPENDS roscpp rospy
  # DEPENDS system_lib
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

option(BUILD_SHARED_LIBS "" ON)

add_library(vmxpi_hal SHARED IMPORTED GLOBAL)
set_target_properties(vmxpi_hal PROPERTIES IMPORTED_LOCATION "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so")
set_target_properties(vmxpi_hal PROPERTIES LINKER_LANGUAGE CXX)

add_library(titandriver_ros src/vmxpi_ros_titan/TitanDriver_ros.cpp)
target_link_libraries(titandriver_ros PRIVATE vmxpi_hal ${catkin_LIBRARIES})
#set_target_properties(titandriver_ros PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
install(
  TARGETS titandriver_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(titandriver_ros_wrapper src/TitanDriver_ros_wrapper.cpp)
target_link_libraries(titandriver_ros_wrapper PRIVATE vmxpi_hal titandriver_ros ${catkin_LIBRARIES})
#set_target_properties(titandriver_ros_wrapper PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
install(
  TARGETS titandriver_ros_wrapper
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


