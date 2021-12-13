#pragma once

#include <ros/ros.h>
#include "VMXPi.h"
#include "vmxpi_utils.h"
#include <std_srvs/Empty.h>
#include <vmxpi_ros/Int.h>

class DigitalOutputROS : public Utils {
	private:
		VMXIO *io; // IO class of the VMXPi
		VMXTime *time; // Time class of the VMXPi
		uint8_t channel_index;
		uint32_t senduS;
		
		VMXResourceHandle digitalout_res_handle;

		// services to enable the ping loop and set the duration of the ping
		ros::ServiceServer sendPing, setuS;
	public:
		DigitalOutputROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel);
		
		// function to send a ping in 100ms intervals
		bool Ping(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		
		// set the ping's duration
		bool SetDuration(vmxpi_ros::Int::Request &req, vmxpi_ros::Int::Response &res);
		
};
