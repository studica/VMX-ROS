#pragma once

#include <ros/ros.h>
#include "VMXPi.h"
#include "vmxpi_utils.h"
#include <chrono>
#include <inttypes.h>
#include <sys/syscall.h>
#include <std_msgs/Float32.h>

class UltrasonicROS : public Utils {
	private:
		VMXIO *io; // IO class of VMXPi
		VMXTime *time; // Time class of VMXPi
		VMXResourceHandle digitalio_res_handles[2]; 
		
		uint8_t channel_index_out, channel_index_in; // channel indexes for DI and DO
		
		ros::Publisher ultrasonic_cm_pub, ultrasonic_in_pub, ultrasonic_raw_pub;
		
		std::thread ultrasonicth;
		std::thread runth;
		
	public:
		UltrasonicROS(ros::NodeHandle *nh, VMXPi *pi, uint8_t channelout, uint8_t channelin);
    	~UltrasonicROS();
		
		double GetDistanceCM(uint32_t diff); // get distance in cm
		double GetDistanceIN(uint32_t diff); // get distance in inches
		uint32_t GetRawValue(); // get raw value

		void Ultrasonic();

    	void Ultrasonic_t();
    	void Run_t();

};
