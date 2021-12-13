#pragma once

#include <ros/ros.h>
#include "VMXPi.h"
#include "vmxpi_utils.h"
#include <std_msgs/Float32.h>
#include <thread>
#include <unistd.h>
#include <sys/syscall.h>

class SharpROS : public Utils {
	private:
		VMXIO *io; // IO class of VMXPi
		VMXResourceHandle accumulator_res_handle;
		uint8_t analog_in_chan_index;
		
		ros::Publisher sharp_dist_pub, raw_voltage_pub;
		
    	std::thread runth;
    
    
	public:
		// first channel input
		SharpROS(ros::NodeHandle *nh, VMXPi *vmx);
		// specify channel input
		SharpROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel);
    	~SharpROS();
		
		double GetIRDistance(); // get distance in cm
		
		double GetRawVoltage(); // get raw voltage
		
   		void Run_t();
};
