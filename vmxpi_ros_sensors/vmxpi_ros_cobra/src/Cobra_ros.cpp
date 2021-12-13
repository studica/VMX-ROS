#include "Cobra_ros.h"

CobraROS::CobraROS(ros::NodeHandle *nh, VMXPi *vmx) :
    io{&vmx->getIO()}, deviceAddress(0x48 /*default address*/), vRef(5.0F /*default voltage*/), mode(CONFIG_MODE_CONT), gain(CONFIG_PGA_2), rate(CONFIG_RATE_1600HZ) {
	
    VMXErrorCode vmxerr;

    VMXChannelInfo i2c_channels[2] = { // find SDA and SCL channels for I2C
	    { VMXChannelInfo(io->GetSoleChannelIndex(VMXChannelCapability::I2C_SDA), VMXChannelCapability::I2C_SDA) },
	    { VMXChannelInfo(io->GetSoleChannelIndex(VMXChannelCapability::I2C_SCL), VMXChannelCapability::I2C_SCL) }};
     
    I2CConfig i2c_cfg;

    if (!io->ActivateDualchannelResource(i2c_channels[0], i2c_channels[1], &i2c_cfg, i2c_res_handle, &vmxerr)) {
      ROS_WARN("Failed to Activate I2C Resource.");
      DisplayVMXError(vmxerr);
    } else {
      ROS_INFO("Successfully Activated I2C Resource with VMXChannels %d and %d",
          i2c_channels[0].index, i2c_channels[1].index);
    }
    
    c0_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c0/raw", 1);
    c1_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c1/raw", 1);
    c2_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c2/raw", 1);
    c3_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c3/raw", 1);
    
    c0_v_pub = nh->advertise<std_msgs::Float32>("cobra/c0/voltage", 1);
    c1_v_pub = nh->advertise<std_msgs::Float32>("cobra/c1/voltage", 1);
    c2_v_pub = nh->advertise<std_msgs::Float32>("cobra/c2/voltage", 1);
    c3_v_pub = nh->advertise<std_msgs::Float32>("cobra/c3/voltage", 1);
    
    runth = std::thread(&CobraROS::Run_t, this);
    
    if (IsConnected())
	    ROS_INFO("Connected");
    else
	    ROS_WARN("Not connected");
}

CobraROS::CobraROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t deviceAddress_) : 
    io{&vmx->getIO()}, deviceAddress(deviceAddress_), vRef(5.0F), mode(CONFIG_MODE_CONT), gain(CONFIG_PGA_2), rate(CONFIG_RATE_1600HZ) {
	
    VMXErrorCode vmxerr;

    VMXChannelInfo i2c_channels[2] = { // find SDA and SCL channels for I2C
	    { VMXChannelInfo(io->GetSoleChannelIndex(VMXChannelCapability::I2C_SDA), VMXChannelCapability::I2C_SDA) },
	    { VMXChannelInfo(io->GetSoleChannelIndex(VMXChannelCapability::I2C_SCL), VMXChannelCapability::I2C_SCL) }};
     
    I2CConfig i2c_cfg;

    if (!io->ActivateDualchannelResource(i2c_channels[0], i2c_channels[1], &i2c_cfg, i2c_res_handle, &vmxerr)) {
      ROS_WARN("Failed to Activate I2C Resource.");
      DisplayVMXError(vmxerr);
    } else {
      ROS_INFO("Successfully Activated I2C Resource with VMXChannels %d and %d",
          i2c_channels[0].index, i2c_channels[1].index);
    }
    
    c0_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c0/raw", 1);
    c1_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c1/raw", 1);
    c2_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c2/raw", 1);
    c3_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c3/raw", 1);
    
    c0_v_pub = nh->advertise<std_msgs::Float32>("cobra/c0/voltage", 1);
    c1_v_pub = nh->advertise<std_msgs::Float32>("cobra/c1/voltage", 1);
    c2_v_pub = nh->advertise<std_msgs::Float32>("cobra/c2/voltage", 1);
    c3_v_pub = nh->advertise<std_msgs::Float32>("cobra/c3/voltage", 1);
    
    runth = std::thread(&CobraROS::Run_t, this);
    
    if (IsConnected())
	    ROS_INFO("Connected");
    else
	    ROS_WARN("Not connected");
}

CobraROS::CobraROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t deviceAddress_, float vRef_) : 
    io{&vmx->getIO()}, deviceAddress(deviceAddress_), vRef(vRef_), mode(CONFIG_MODE_CONT), gain(CONFIG_PGA_2), rate(CONFIG_RATE_1600HZ) {
	
    VMXErrorCode vmxerr;

    VMXChannelInfo i2c_channels[2] = { // find SDA and SCL channels for I2C
	    { VMXChannelInfo(io->GetSoleChannelIndex(VMXChannelCapability::I2C_SDA), VMXChannelCapability::I2C_SDA) },
	    { VMXChannelInfo(io->GetSoleChannelIndex(VMXChannelCapability::I2C_SCL), VMXChannelCapability::I2C_SCL) }};
     
    I2CConfig i2c_cfg;

    if (!io->ActivateDualchannelResource(i2c_channels[0], i2c_channels[1], &i2c_cfg, i2c_res_handle, &vmxerr)) {
      ROS_WARN("Failed to Activate I2C Resource.");
      DisplayVMXError(vmxerr);
    } else {
      ROS_INFO("Successfully Activated I2C Resource with VMXChannels %d and %d",
          i2c_channels[0].index, i2c_channels[1].index);
    }
    
    c0_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c0/raw", 1);
    c1_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c1/raw", 1);
    c2_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c2/raw", 1);
    c3_raw_pub = nh->advertise<std_msgs::Int16>("cobra/c3/raw", 1);
    
    c0_v_pub = nh->advertise<std_msgs::Float32>("cobra/c0/voltage", 1);
    c1_v_pub = nh->advertise<std_msgs::Float32>("cobra/c1/voltage", 1);
    c2_v_pub = nh->advertise<std_msgs::Float32>("cobra/c2/voltage", 1);
    c3_v_pub = nh->advertise<std_msgs::Float32>("cobra/c3/voltage", 1);
    
    runth = std::thread(&CobraROS::Run_t, this);
    
    if (IsConnected())
	    ROS_INFO("Connected");
    else
	    ROS_WARN("Not connected");
}


CobraROS::~CobraROS() {
    VMXErrorCode vmxerr;
    if (!io->DeactivateResource(i2c_res_handle, &vmxerr))
      DisplayVMXError(vmxerr);
    
    runth.join();
}

bool CobraROS::IsConnected() {
    VMXErrorCode vmxerr;
    uint8_t buffer = 0;
	
    if (io->I2C_Read(i2c_res_handle, deviceAddress, 0, &buffer, 1, &vmxerr)) // to fix, I2C_Read function currently does not work
	    return true;
    else {
      DisplayVMXError(vmxerr);
	    return false;
    }
}

int CobraROS::GetRawValue(uint8_t channel) {
    return GetSingle(channel);
}

float CobraROS::GetVoltage(uint8_t channel) {
    float raw = GetSingle(channel), mV = vRef/2048;
    return raw*mV;
}

int CobraROS::GetSingle(uint8_t channel) {
    if (channel > 3)
	    return 0;
    
    int config = CONFIG_OS_SINGLE | mode | rate | gain; // OR the config options together
    switch(channel) { // this allows the config data to be specific to each channel
	    case(0):
        config |= CONFIG_MUX_SINGLE_0;
        break;
	    case(1):
        config |= CONFIG_MUX_SINGLE_1;
        break;
	    case(2):
	      config |= CONFIG_MUX_SINGLE_2;
	      break;
	    case(3):
	      config |= CONFIG_MUX_SINGLE_3;
	      break;
    }
    uint8_t raw[2] = {(uint8_t) (config >> 8), (uint8_t) (config & 0xFF)};
    uint8_t buffer[2];
    VMXErrorCode vmxerr;

    if (!io->I2C_Write(i2c_res_handle, deviceAddress, // send the config data to the cobra
			    POINTER_CONFIG, raw, (int32_t)sizeof(raw), 
			    &vmxerr)) {
	    ROS_WARN("I2C send failed");
      DisplayVMXError(vmxerr);
    }
	
    uint8_t data[2] = {POINTER_CONVERT, POINTER_CONVERT};
    
    if (!io->I2C_Transaction(i2c_res_handle, deviceAddress, // write to the register in order to receive the raw value back
				data, (uint16_t)sizeof(data),
				buffer, (uint16_t)sizeof(buffer),
				&vmxerr)) {
	    ROS_WARN("I2C Transaction failed");
      DisplayVMXError(vmxerr);
    }
	
    int val = (int)((buffer[0]<<8) & 0xFF00) | (buffer[1] & 0xFF); // since the data is little-endian, shift data to get correct value
    //ROS_INFO_STREAM("channel " << channel << ": " << (val >> 4));
    return val>>4;
}

void CobraROS::Run_t() {
    ros::Rate r(50);
    ROS_INFO_STREAM("Cobra pub thread: " << syscall(SYS_gettid));
    while (ros::ok()) {
      std_msgs::Int16 raw_msg;
      std_msgs::Float32 v_msg;

      raw_msg.data = GetRawValue(0);
      v_msg.data = GetVoltage(0);
      c0_raw_pub.publish(raw_msg);
      c0_v_pub.publish(v_msg);
    
      raw_msg.data = GetRawValue(1);
      v_msg.data = GetVoltage(1);
      c1_raw_pub.publish(raw_msg);
      c1_v_pub.publish(v_msg);
    
      raw_msg.data = GetRawValue(2);
      v_msg.data = GetVoltage(2);
      c2_raw_pub.publish(raw_msg);
      c2_v_pub.publish(v_msg);
    
      raw_msg.data = GetRawValue(3);
      v_msg.data = GetVoltage(3);
      c3_raw_pub.publish(raw_msg);
      c3_v_pub.publish(v_msg);
      r.sleep();
    }
}
