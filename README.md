# Description

This is the Studica Open Source ROS repository, allowing ROS functionality for a variety of Studica's hardware. Information and instructions on using this library can be found at https://docs.wsr.studica.com/en/latest/index.html. 

# Package descriptions

Package name | Description
------------ | -------------
vmxpi_ros_bringup | A test_node.cpp (example) and main.cpp, this is what's used to launch the ROS node
vmxpi_ros_titan | ROS driver for the Titan motor controller
vmxpi_ros_navx | ROS driver for onboard NavX sensor
vmxpi_ros_sensors | ROS drivers for other sensors (Cobra, Sharp IR sensor, Ultrasonic Sensor)
vmxpi_ros_servo | ROS driver for Servos
vmxpi_ros_io | ROS driver for VMXPi pins (Digital IO)
vmxpi_ros_cam | ROS driver for OpenCV (contains a demo)
vmxpi_ros | Contains utilities, msg and srv files
