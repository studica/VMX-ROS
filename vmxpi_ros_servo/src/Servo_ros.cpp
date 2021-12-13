#include "Servo_ros.h"

uint16_t ServoROS::mapAngle (double angle)
{
	/**
	 * 2.5% (5000) = 0.5ms -150 degrees (CW Full speed) Value to put 125
     * 7.5% (5000) = 1.5ms 0 degrees (Stopped) Value to put 375
     * 12.5% (5000) = 2.5ms +150 degrees (CCW Full speed) Value to put 625
    **/
   	//With the updated duty cycle range the accuracy is now 0.6 degrees per step
  
  if (angle < minangle)
    angle = minangle;
  else if (angle > maxangle)
    angle = maxangle;
    
  double val = ((angle - (-150)) * (625 - 125) / (150 - (-150)) + 125);
	return (uint16_t) val;
}

ServoROS::ServoROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel) :
	io{&vmx->getIO()}, channel_index(channel) {
	VMXErrorCode vmxerr;
	
	PWMGeneratorConfig pwmgen_cfg(50); // servo is 50hz
	pwmgen_cfg.SetMaxDutyCycleValue(5000); // 50mS / 1uS = 5000

	res_port_index = 0;

	if (!io->ActivateSinglechannelResource(VMXChannelInfo(channel_index, VMXChannelCapability::PWMGeneratorOutput), &pwmgen_cfg, 
		pwmgen_res_handle, &vmxerr)) {
		ROS_WARN("Error Activating Singlechannel Resource PWM Generator for Channel index %d.", channel_index);
    DisplayVMXError(vmxerr);
	} else {
		ROS_INFO("Successfully Activated PWMGenerator Resource %d with VMXChannel %d", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle), channel_index);
		if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle, res_port_index, mapAngle(0), &vmxerr)) {
			ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle), res_port_index);
      DisplayVMXError(vmxerr);
		}
	}
 
	std::string topic_name = "channel/" + std::to_string(channel_index) + "/servo/";
	servo_angle_pub = nh->advertise<std_msgs::Float32>(topic_name + "angle", 1);
	setangle = nh->advertiseService(topic_name + "set_angle", &ServoROS::SetAngle, this);
	
	runth = std::thread(&ServoROS::Run_t, this);
}

ServoROS::~ServoROS() {
  runth.join();
}
double ServoROS::GetAngle() const { return angle_; }

bool ServoROS::SetAngle(vmxpi_ros::Float::Request &req, vmxpi_ros::Float::Response &res) { 
  VMXErrorCode vmxerr;
  
  // PWM uses duty cycle, for more info on this: https://www.allaboutcircuits.com/technical-articles/introduction-to-microcontroller-timers-pwm-timers/
  if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle, res_port_index, mapAngle(req.data), &vmxerr)) {
		ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle), res_port_index);
    DisplayVMXError(vmxerr);
    return false;
  }
  angle_ = req.data; 
  return true;
}

double ServoROS::GetMinAngle() const { return minangle; }
 
double ServoROS::GetMaxAngle() const { return maxangle; }

void ServoROS::Run_t() {
  ros::Rate r(50);
  ROS_INFO_STREAM("Servo pub thread: " << syscall(SYS_gettid));
  while(ros::ok()) {
    std_msgs::Float32 msg;
    msg.data = angle_;
    servo_angle_pub.publish(msg);
    r.sleep();
  }
}




