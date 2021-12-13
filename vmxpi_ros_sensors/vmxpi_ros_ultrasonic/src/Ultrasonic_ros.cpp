#include "Ultrasonic_ros.h"

UltrasonicROS::UltrasonicROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channelout, uint8_t channelin) :
	io{&vmx->getIO()}, time{&vmx->getTime()}, channel_index_out(channelout), channel_index_in(channelin) {	
	VMXErrorCode vmxerr;
		
	uint8_t digital_output_res_handle_index = 0;

	DIOConfig dio_config(DIOConfig::OutputMode::PUSHPULL);
	
	if (!io->ActivateSinglechannelResource(VMXChannelInfo(channel_index_out, VMXChannelCapability::DigitalOutput), &dio_config, 
		digitalio_res_handles[digital_output_res_handle_index], &vmxerr)) {
		printf("Error Activating Singlechannel Resource DIO for Channel index %d.\n", channel_index_out);
    DisplayVMXError(vmxerr);
	} else {
		ROS_INFO("Digital Output Channel %d activated on Resource type %d, index %d\n", channel_index_out,
						EXTRACT_VMX_RESOURCE_TYPE(digitalio_res_handles[digital_output_res_handle_index]),
						EXTRACT_VMX_RESOURCE_INDEX(digitalio_res_handles[digital_output_res_handle_index]));
	}
	
	digital_output_res_handle_index++;
	
	DIOConfig dio_config2;
	if (!io->ActivateSinglechannelResource(VMXChannelInfo(channel_index_in, VMXChannelCapability::DigitalInput), &dio_config2, 
		digitalio_res_handles[digital_output_res_handle_index], &vmxerr)) {
		printf("Error Activating Singlechannel Resource DIO for Channel index %d.\n", channel_index_in);
    DisplayVMXError(vmxerr);
	} else {
		ROS_INFO("Digital Input Channel %d activated on Resource type %d, index %d\n", channel_index_in,
						EXTRACT_VMX_RESOURCE_TYPE(digitalio_res_handles[digital_output_res_handle_index]),
						EXTRACT_VMX_RESOURCE_INDEX(digitalio_res_handles[digital_output_res_handle_index]));
	}
	
	if (!io->DIO_Set(digitalio_res_handles[0], false, &vmxerr)) 
		DisplayVMXError(vmxerr);
	
	std::string topic_name = "channel/" + std::to_string(channelin) + "/ultrasonic/";
	ultrasonic_cm_pub = nh->advertise<std_msgs::Float32>(topic_name + "dist/cm", 1);
	ultrasonic_in_pub = nh->advertise<std_msgs::Float32>(topic_name + "dist/inch", 1);
	ultrasonic_raw_pub = nh->advertise<std_msgs::Float32>(topic_name + "raw", 1);
	
	runth = std::thread(&UltrasonicROS::Run_t, this);
}

UltrasonicROS::~UltrasonicROS() {
  ultrasonicth.join();
  runth.join();
}

double UltrasonicROS::GetDistanceCM(uint32_t diff) {
	return (double)diff / 58;
}

double UltrasonicROS::GetDistanceIN(uint32_t diff) {
	return (double)diff / 148;
}

uint32_t UltrasonicROS::GetRawValue() {
	VMXErrorCode vmxerr;
	bool state;
	uint32_t diff, begin = 0, end = 0;

	while (io->DIO_Get(digitalio_res_handles[1], state, &vmxerr)) {
		if (state) {
			begin = time->GetCurrentMicroseconds();
			//ROS_INFO("begin: %" PRIu64"\n", begin);
			break;
		}
		
	}
		
	while (io->DIO_Get(digitalio_res_handles[1], state, &vmxerr)) { 
		if (!state) {
			end = time->GetCurrentMicroseconds();
			//ROS_INFO("end: %" PRIu64"\n", end);
			break;
		}
	}
	
	diff = end - begin;
	//ROS_INFO("diff: %" PRIu32 "\n", diff);
	return diff;
}

void UltrasonicROS::Run_t() {
	ros::Rate r(50);
    ROS_INFO_STREAM("Ultrasonic pub thread: " << syscall(SYS_gettid));
    while (ros::ok()) {
	uint32_t diff = this -> GetRawValue();
	std_msgs::Float32 msg;
		
        msg.data = diff;
        ultrasonic_raw_pub.publish(msg);
        
        ROS_INFO_STREAM("Raw Distance: " << roundf(msg.data) << "ms" << "\n");
        
        msg.data = GetDistanceCM(diff);
        ultrasonic_cm_pub.publish(msg);
            
        ROS_INFO_STREAM("Distance in Centimeters: " << roundf(msg.data) << "cm" << "\n");
        
        msg.data = GetDistanceIN(diff);
        ultrasonic_in_pub.publish(msg);
        
        ROS_INFO_STREAM("Distance in Inches: " << roundf(msg.data) << "in" << "\n");
        r.sleep();
    }
	
}

void UltrasonicROS::Ultrasonic() {
	ultrasonicth = std::thread(&UltrasonicROS::Ultrasonic_t, this);
}

void UltrasonicROS::Ultrasonic_t() {
	VMXErrorCode vmxerr;
	uint32_t senduS = 10; // The duration of the pulse
	while (ros::ok()) {
		if(io->DIO_Pulse(digitalio_res_handles[0], true, senduS, &vmxerr)) {
			//ROS_INFO("Ultrasonic");
		} else {
			DisplayVMXError(vmxerr);
		}

		time->DelayMilliseconds(100);
	}
}
