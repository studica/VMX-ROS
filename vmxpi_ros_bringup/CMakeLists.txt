cmake_minimum_required(VERSION 3.0.2)
project(vmxpi_ros_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  vmxpi_ros
  vmxpi_ros_titan
  vmxpi_ros_cobra
  vmxpi_ros_sharp
  vmxpi_ros_ultrasonic
  vmxpi_ros_navx
  vmxpi_ros_servo
  vmxpi_ros_io

)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  cfg/motor_speed.cfg
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
#  LIBRARIES vmxpi_ros_bringup
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ../vmxpi_ros_titan/include
  ../vmxpi_ros_navx/include
  ../vmxpi_ros_sensors/vmxpi_ros_cobra/include
  ../vmxpi_ros_sensors/vmxpi_ros_sharp/include
  ../vmxpi_ros_sensors/vmxpi_ros_ultrasonic/include
  ../vmxpi_ros_servo/include
  ../vmxpi_ros_utils/include
  ../vmxpi_ros_io/include
  ../vmxpi_ros/include
  /usr/local/include/vmxpi
)

add_library(vmxpi_hal SHARED IMPORTED GLOBAL)
set_target_properties(vmxpi_hal PROPERTIES IMPORTED_LOCATION "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so")

add_library(navx_ros_wrapper SHARED IMPORTED GLOBAL)
# set_target_properties(navx_ros_wrapper PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libnavx_ros_wrapper.so")
set_target_properties(navx_ros_wrapper PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libnavx_ros_wrapper.so)

add_library(titandriver_ros_wrapper SHARED IMPORTED GLOBAL)
# set_target_properties(titandriver_ros_wrapper PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libtitandriver_ros_wrapper.so")
set_target_properties(titandriver_ros_wrapper PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libtitandriver_ros_wrapper.so)

add_library(titandriver_ros SHARED IMPORTED GLOBAL)
# set_target_properties(titandriver_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libtitandriver_ros.so")
set_target_properties(titandriver_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libtitandriver_ros.so)

add_library(cobra_ros SHARED IMPORTED GLOBAL)
set_target_properties(cobra_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libcobra_ros.so)

add_library(sharp_ros SHARED IMPORTED GLOBAL)
# set_target_properties(sharp_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libsharp_ros.so")
set_target_properties(sharp_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libsharp_ros.so)

add_library(servo_ros SHARED IMPORTED GLOBAL)
# set_target_properties(servo_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libservo_ros.so")
set_target_properties(servo_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libservo_ros.so)

add_library(ultrasonic_ros SHARED IMPORTED GLOBAL)
# set_target_properties(ultrasonic_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libultrasonic_ros.so")
set_target_properties(ultrasonic_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libultrasonic_ros.so)

add_library(iowd_ros SHARED IMPORTED GLOBAL)
# set_target_properties(iowd_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libiowd_ros.so")
set_target_properties(iowd_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libiowd_ros.so)

add_library(digitalin_ros SHARED IMPORTED GLOBAL)
# set_target_properties(digitalin_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libdigitalin_ros.so")
set_target_properties(digitalin_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libdigitalin_ros.so)

add_library(digitalout_ros SHARED IMPORTED GLOBAL)
# set_target_properties(digitalout_ros PROPERTIES IMPORTED_LOCATION "/home/pi/catkin_ws/devel/lib/libdigitalout_ros.so")
set_target_properties(digitalout_ros PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/../../../devel/lib/libdigitalout_ros.so)

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node PRIVATE
	vmxpi_hal 
	navx_ros_wrapper 
	titandriver_ros 
	titandriver_ros_wrapper 
	cobra_ros 
	sharp_ros
	servo_ros 
	ultrasonic_ros
  	iowd_ros
	digitalin_ros
	digitalout_ros
	${catkin_LIBRARIES}


#  ${PROJECT_SOURCE_DIR}/../../../devel/lib/libnavx_ros_wrapper.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libtitandriver_ros_wrapper.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libtitandriver_ros.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libcobra_ros.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libsharp_ros.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libservo_ros.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libultrasonic_ros.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libiowd_ros.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libdigitalin_ros.so
#  ${PROJECT_SOURCE_DIR}/../../devel/lib/libdigitalout_ros.so
  
) 
add_dependencies(test_node 
  	navx_ros_wrapper 
	titandriver_ros 
	titandriver_ros_wrapper 
	cobra_ros 
	sharp_ros
	servo_ros 
	ultrasonic_ros
  	iowd_ros
	digitalin_ros
	digitalout_ros
  	${PROJECT_NAME}_gencfg)


add_executable(main_node src/main.cpp)
target_link_libraries(main_node PRIVATE
	vmxpi_hal 
	navx_ros_wrapper 
	titandriver_ros 
	titandriver_ros_wrapper 
	cobra_ros 
	sharp_ros
	servo_ros 
	ultrasonic_ros
  	iowd_ros
	digitalin_ros
	digitalout_ros
	${catkin_LIBRARIES}
)
add_dependencies(main_node 
  	navx_ros_wrapper 
	titandriver_ros 
	titandriver_ros_wrapper 
	cobra_ros 
	sharp_ros
	servo_ros 
	ultrasonic_ros
  	iowd_ros
	digitalin_ros
	digitalout_ros
  	${PROJECT_NAME}_gencfg)
