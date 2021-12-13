cmake_minimum_required(VERSION 3.0.2)
project(vmxpi_ros_cam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  vmxpi_ros
  dynamic_reconfigure
)

find_package(OpenCV)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/opencv_settings.cfg
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
#  LIBRARIES vmxpi_ros_cam
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs
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
  ${OpenCV_INCLUDE_DIRS}
  /home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros_utils/include
  /home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros_io/include
  /home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/include
  /usr/local/include/vmxpi
)

add_executable(camera_node src/camera_ros.cpp)
target_link_libraries(camera_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(opencv_node src/opencv_ros.cpp)
target_link_libraries(opencv_node 
		${OpenCV_LIBRARIES} 
		${catkin_LIBRARIES})
add_dependencies(opencv_node 
		${PROJECT_NAME}_gencfg)