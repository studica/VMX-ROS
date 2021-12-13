cmake_minimum_required(VERSION 3.0.2)
project(vmxpi_ros_cobra)

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

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES vmxpi_ros_cobra
#  CATKIN_DEPENDS actionlib roscpp rospy
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
  ../../vmxpi_ros/include
)

add_library(vmxpi_hal SHARED IMPORTED GLOBAL)
set_target_properties(vmxpi_hal PROPERTIES IMPORTED_LOCATION "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so")
set_target_properties(vmxpi_hal PROPERTIES LINKER_LANGUAGE CXX)

add_library(cobra_ros src/Cobra_ros.cpp)
target_link_libraries(cobra_ros PRIVATE vmxpi_hal ${catkin_LIBRARIES})
install(
  TARGETS cobra_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

