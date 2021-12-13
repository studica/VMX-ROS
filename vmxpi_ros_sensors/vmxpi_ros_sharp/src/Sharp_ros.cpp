#include "Sharp_ros.h"

SharpROS::SharpROS(ros::NodeHandle *nh, VMXPi *vmx) : 
  io{&vmx->getIO()} {
	
	VMXChannelIndex anin_channel; // default channel is the first analog channel
	io->GetNumChannelsByType(VMXChannelType::AnalogIn, anin_channel);
	analog_in_chan_index = anin_channel;
	
	VMXErrorCode vmxerr;
	AccumulatorConfig accum_config;
	accum_config.SetNumAverageBits(9);
	accum_config.SetEnableAccumulationCounter(true);
	accum_config.SetAccumulationCounterCenter(0);
	accum_config.SetAccumulationCounterDeadband(4); // configure the voltage accumulator before activating 
	
	if (!io->ActivateSinglechannelResource(VMXChannelInfo(analog_in_chan_index, VMXChannelCapability::AccumulatorInput),
		&accum_config, accumulator_res_handle, &vmxerr)) {
    	DisplayVMXError(vmxerr);
  	} else {
    ROS_INFO("Analog Input Channel %d activated on Resource type %d, index %d", analog_in_chan_index,
		EXTRACT_VMX_RESOURCE_TYPE(accumulator_res_handle),
		EXTRACT_VMX_RESOURCE_INDEX(accumulator_res_handle));
  	}
	
  	std::string topic_name = "channel/" + std::to_string(analog_in_chan_index) + "/sharp_ir/";
	sharp_dist_pub = nh->advertise<std_msgs::Float32>(topic_name + "dist", 1);
	raw_voltage_pub = nh->advertise<std_msgs::Float32>(topic_name + "raw", 1);
	
	runth = std::thread(&SharpROS::Run_t, this);
}

SharpROS::SharpROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel) : 
	io{&vmx->getIO()} {
	
	if (channel < 22 || channel > 25) // analog channels are 22 - 25, see https://docs.wsr.studica.com/en/latest/docs/VMX/wpi-channel-addressing.html
		ROS_WARN("Invalid analog input channel");
	else
	  analog_in_chan_index = channel;
	
	VMXErrorCode vmxerr;
	AccumulatorConfig accum_config;
	accum_config.SetNumAverageBits(9);
	accum_config.SetEnableAccumulationCounter(true);
	accum_config.SetAccumulationCounterCenter(0);
	accum_config.SetAccumulationCounterDeadband(4); // configure the voltage accumulator before activating
	
	if (!io->ActivateSinglechannelResource(VMXChannelInfo(analog_in_chan_index, VMXChannelCapability::AccumulatorInput),
		&accum_config, accumulator_res_handle, &vmxerr)) {
    DisplayVMXError(vmxerr);
  	} else {
    ROS_INFO("Analog Input Channel %d activated on Resource type %d, index %d", analog_in_chan_index,
		EXTRACT_VMX_RESOURCE_TYPE(accumulator_res_handle),
		EXTRACT_VMX_RESOURCE_INDEX(accumulator_res_handle));
  	}	
      
	
	std::string topic_name = "channel/" + std::to_string(analog_in_chan_index) + "/sharp_ir/";
	sharp_dist_pub = nh->advertise<std_msgs::Float32>(topic_name + "dist", 1);
	raw_voltage_pub = nh->advertise<std_msgs::Float32>(topic_name + "raw", 1);
	
	runth = std::thread(&SharpROS::Run_t, this);
}

SharpROS::~SharpROS() {
  	runth.join();
}

double SharpROS::GetIRDistance() {
	return(pow(GetRawVoltage(), -1.2045)) * 27.726; // convert the voltage read
}

double SharpROS::GetRawVoltage() {
	VMXErrorCode vmxerr;
	float voltage;
 
	if(!io->Accumulator_GetAverageVoltage(accumulator_res_handle, voltage, &vmxerr)) // since the sharp IR sensor is analog, get the average voltage 
    	DisplayVMXError(vmxerr);
	
	return voltage;
}

void SharpROS::Run_t() {
  ros::Rate r(50);
  ROS_INFO_STREAM("Sharp pub thread: " << syscall(SYS_gettid));
  while (ros::ok()) {
        std_msgs::Float32 msg;
        msg.data = GetIRDistance();
        sharp_dist_pub.publish(msg);
        msg.data = GetRawVoltage();
        raw_voltage_pub.publish(msg);
        r.sleep();
    }
}
