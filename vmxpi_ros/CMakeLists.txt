cmake_minimum_required(VERSION 3.0.2)
project(vmxpi_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LimitSwitch.msg
  TitanInfo.msg
  UniqueID.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Int.srv
  IntRes.srv
  Float.srv
  FloatRes.srv
  MotorSpeed.srv
  StopMode.srv
  StringRes.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  std_srvs
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
#  LIBRARIES vmxpi_ros
#  CATKIN_DEPENDS roscpp rospy
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
)

add_library(vmxpi_hal SHARED IMPORTED GLOBAL)
set_target_properties(vmxpi_hal PROPERTIES IMPORTED_LOCATION "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so")
set_target_properties(vmxpi_hal PROPERTIES LINKER_LANGUAGE CXX)


