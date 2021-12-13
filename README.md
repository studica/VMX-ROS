# Description

This is the Studica Open Source ROS repository, allowing ROS functionality for a variety of Studica's hardware. Compared to the current codebase, this directly uses the VMXPi HAL created by Scott from kauailabs. More information on the VMXPi HAL API can be found at https://www.kauailabs.com/public_files/vmx-pi/apidocs/hal_cpp/html/class_v_m_x_pi.html. 

To see indepth examples of the VMXPi HAL, your RPi has a lot in /usr/local/src/vmxpi/

A lot of useful information on the components used can be found at https://docs.wsr.studica.com/en/latest/index.html. 

# Todo

As of now, all ROS libraries for the components have been drafted. Currently, to push an alpha version of this, a list of what needs to be done is:
* Fix the Cobra's I2C communication with the VMXPi -- currently, values are completely incorrect or double than intended
# Instructions on running package

1. Enter the catkin_ws directory with `cd catkin_ws/`, then run `catkin build` 
  
2. In a separate terminal tab or window, run `rosmaster --core`. Compared to the usual `roscore`, this does not take as much load on the CPU. 
  
3. In another separate terminal, switch to `root` by running `sudo su`. This will require you to source the setup files needed to run ROS.
  
4. To source the setup files, run `. /opt/ros/noetic/setup.bash` and `. catkin_ws/devel/setup.bash`

5. In your root terminal, you can now launch the package. To do so, run `roslaunch vmxpi_ros_bringup wrapper.launch`

# Running ROS Qt Dashboard Perspective File

1. There are two ways of loading the `vmxpi_ros_rqt.perspective` file:
* By launching the rqt gui
* From the command line

2. RQT gui:
* Run `rqt` in the command line ensuring `roscore` is running in the background
* Navigate to the Perspectives tab
* Select `Import...` from the dropdown
* Navigate to the location of the `vmxpi_ros_rqt.perspective` file and open it

3. Command Line:
* Run `roscore`
* Open another terminal and run `rqt --perspective-file "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros_rqt.perspective"`

# Running package from scratch

1. If you haven't already, run `installNoetic.sh` to install ROS Noetic. For more information, visit http://wiki.ros.org/noetic/Installation/Debian

2. Download catkin tools from https://catkin-tools.readthedocs.io/en/latest/installing.html, then run `sudo apt install python3-catkin-tools python3-osrf-pycommon` to finalize the install.

3. Create a directory `catkin_ws` by running `mkdir -p ~/catkin_ws/src`, preferrably in `/home/pi`. Then `cd catkin_ws` and run `catkin init` to initialize the workspace environment.

4. Next, run `catkin build`, then clone this repo into the `src` folder.

5. Permanently source the setup.bash files by running the following: `echo "source /opt/ros/noetic/setup.bash" >> ~/.profile`, `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`, `echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.profile` and `echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.bashrc`.

6. Close the terminal and open a new one.

7. Navigate to the work space `cd catkin_ws/src`

8. Change the name of the `ros-dev` folder to `vmxpi_ros`

9. To build the packages run `catkin build -cs`. Note, this may take a while as the command builds all the packages in the catkin workspace.

10. With everything built, you can begin running the node. The VMXPi HAL uses the pigpio library, which unfortunately can only be used in one process. Thus, everything that interfaces with the VMXPi must be run on the same executable. Run `frcKillRobot.sh` to kill the robot manager used for WPILib.

11. In a new terminal, run `sudo su` to run commands as root.

12. In root run `echo "source /opt/ros/noetic/setup.bash" >> ~/.profile`, `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`, `echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.profile` and `echo "source /home/pi/catkin_ws/devel/setup.bash" >> ~/.bashrc`.

13. Close the terminal and reopen it. Navigate to `cd catkin_ws/src` and run `sudo su`

14. In another terminal, run `roscore` to allow ROS communication.

15. Now, run `roslaunch vmxpi_ros_bringup wrapper.launch` to start it.

# Package descriptions

Package name | Description
------------ | -------------
vmxpi_ros_bringup | Essentially main.cpp, this is what's used to launch the ROS node
vmxpi_ros_titan | ROS for Titan motor controller
vmxpi_ros_navx | ROS for onboard navX sensor
vmxpi_ros_sensors | ROS for other sensors (Cobra line detector, Sharp IR sensor, Ultrasonic ping)
vmxpi_ros_servo | ROS for servos
vmxpi_ros_io | ROS for VMXPi pins (digital IO)
vmxpi_ros_cam | ROS for OpenCV and raspicam (this is only for demo)
vmxpi_ros | Contains utilities, msg and srv files
