#include "DO_ros.h"

DigitalOutputROS::DigitalOutputROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel) :
	io{&vmx->getIO()}, time{&vmx->getTime()}, channel_index(channel), senduS(10 /* default ping duration */) {
	
	VMXErrorCode vmxerr;
	DIOConfig dio_config(DIOConfig::OutputMode::PUSHPULL); // DO configuration

	if (!io->ActivateSinglechannelResource(VMXChannelInfo(channel_index, VMXChannelCapability::DigitalOutput), &dio_config, 
		digitalout_res_handle, &vmxerr)) {
		ROS_INFO("Error Activating Singlechannel Resource DIO for Channel index %d.", channel_index);
    	DisplayVMXError(vmxerr);
	} else {
		ROS_INFO("Digital Output Channel %d activated on Resource type %d, index %d", channel_index,
						EXTRACT_VMX_RESOURCE_TYPE(digitalout_res_handle),
						EXTRACT_VMX_RESOURCE_INDEX(digitalout_res_handle));
	}
	
	std::string topic_name = "channel/" + std::to_string(channel_index) + "/digital_out/";
	sendPing = nh->advertiseService(topic_name + "send_ping", &DigitalOutputROS::Ping, this);
	setuS = nh->advertiseService(topic_name + "set_ping_duration", &DigitalOutputROS::SetDuration, this);
}

bool DigitalOutputROS::Ping(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
	VMXErrorCode vmxerr;
	if(io->DIO_Pulse(digitalout_res_handle, true, senduS, &vmxerr)) { // send a ping out with duration senduS
		//ROS_INFO("Ping"); // debug
	} else {
		DisplayVMXError(vmxerr);
	}

	time->DelayMilliseconds(100);
	return true;
}

bool DigitalOutputROS::SetDuration(vmxpi_ros::Int::Request &req, vmxpi_ros::Int::Response &res) {
	senduS = req.data;
	return true;
}
