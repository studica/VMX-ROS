#pragma once

#include <ros/ros.h>
#include "VMXPi.h"
#include "vmxpi_utils.h"
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <sys/syscall.h>

#include <std_msgs/Float32.h>
#include <vmxpi_ros/Float.h>

class ServoROS : public Utils {
	private:
		VMXIO *io; // IO class of VMXPi

		VMXResourceHandle pwmgen_res_handle;
		VMXResourcePortIndex res_port_index;
		
		ros::Publisher servo_angle_pub;
    	ros::ServiceServer setangle; // service to set the angle
    
    	double maxangle = 150, minangle = -150;
		
		uint16_t mapAngle(double angle); // convert user's input of degrees into a value readable by the HAL
		uint8_t channel_index;
		
		double angle_ = 0;
   
   		std::thread runth;
    
	public:
		ServoROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel);
    	~ServoROS();
		
		bool SetAngle(vmxpi_ros::Float::Request &req, vmxpi_ros::Float::Response &res); 
   
		double GetAngle() const; // get current angle
    	double GetMinAngle() const; // get the servo's min angle
    	double GetMaxAngle() const; // get the servo's max angle
	
    	void Run_t();
};
