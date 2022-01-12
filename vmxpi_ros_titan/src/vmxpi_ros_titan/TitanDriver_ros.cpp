#include "../../include/TitanDriver_ros.h"
#include <stdint.h>

// Run loop on separate thread to disable/enable Titan
void TitanDriver::Run() {
    ROS_INFO_STREAM("Titan run thread: " << syscall(SYS_gettid));
    
    VMXCANMessage msg;
    VMXErrorCode vmxerr;
    uint8_t data[8];
    msg.dataSize = 8;
    msg.setData(data, 8);
    ros::Rate r(100);
    while(ros::ok()) {
        if(enabled)
            msg.messageID = TitanMessageID::ENABLED_FLAG + deviceID;
        else
            msg.messageID = TitanMessageID::DISABLED_FLAG + deviceID;
        if (!can->SendMessage(msg, 0, &vmxerr))
            DisplayVMXError(vmxerr);
        
        r.sleep();
    }
}

TitanDriver::TitanDriver(int deviceID_set, VMXCAN *can_) 
    : can(can_), enabled(true) {
    if (deviceID_set >0 && deviceID_set < 64) { // Ensure device ID is valid (1-63)
        deviceID = deviceID_set;
        ROS_INFO_STREAM("Titan Device ID: " << deviceID_set);
        // If no frequency is provided, motors are configured with default freq
        ConfigureMotor(0); 
        ConfigureMotor(1);
        ConfigureMotor(2);
        ConfigureMotor(3);
        
        ConfigureEncoder(0, 51.0, 1464.0, 1/1.0);
        ConfigureEncoder(1, 51.0, 1464.0, 1/1.0);
        ConfigureEncoder(2, 51.0, 1464.0, 1/1.0);
        ConfigureEncoder(3, 51.0, 1464.0, 1/1.0);
    }
    VMXErrorCode vmxerr;
    try {
        if (CheckCANStatus(vmxerr)) {
            ROS_INFO("VMXCAN opened");
            can->SetMode(VMXCAN::VMXCAN_NORMAL, &vmxerr);
            can->FlushTxFIFO(&vmxerr);
            // Open receive stream for CAN Blackboard, which ensures latest packet is received
            if (!can->OpenReceiveStream(canrxhandles[0], 0x0, 0x0, 100, &vmxerr)) {
                ROS_WARN("Error opening CAN RX Stream, handle: %d\n", canrxhandles[0]);
                DisplayVMXError(vmxerr);
            } else {
                ROS_INFO("Opened CAN RX Stream, handle: %d\n", canrxhandles[0]);
                if(can->EnableReceiveStreamBlackboard(canrxhandles[0], true, &vmxerr)) {
                    ROS_INFO("Enable Blackboard on CAN RX Stream, handle: %d\n", canrxhandles[0]);
                } else {
                    ROS_WARN("Error Enabling Blackboard on CAN RX Stream, handle; %d,\n", canrxhandles[0]);
                    DisplayVMXError(vmxerr);
                }
            }
            if (!CheckCANStatus(vmxerr))
                ROS_WARN("CAN ERROR");
        }
    }
    catch (const std::exception& ex) {
        ROS_INFO("Caught exception: %s\n", ex.what());
    }
    // Start separate run loop thread to prevent blocking sent packets
    runth = std::thread(&TitanDriver::Run, this);
}

TitanDriver::TitanDriver(int deviceID_set, int frequency_set, VMXCAN *can_) 
    : can(can_), enabled(true) {
    if (deviceID_set >0 && deviceID_set < 64) { // Ensure device ID is valid (1-63)
        deviceID = deviceID_set;
        ROS_INFO_STREAM("Titan Device ID: " << deviceID_set);
        if (frequency_set >= 0 && frequency_set <= 20000) { // Ensure frequency is valid
            frequency = frequency_set;
            ConfigureMotor(0);
            ConfigureMotor(1);
            ConfigureMotor(2);
            ConfigureMotor(3);
            
            ConfigureEncoder(0, 51.0, 1464.0, 1/1.0);
            ConfigureEncoder(1, 51.0, 1464.0, 1/1.0);
            ConfigureEncoder(2, 51.0, 1464.0, 1/1.0);
            ConfigureEncoder(3, 51.0, 1464.0, 1/1.0);
        }
    }
    VMXErrorCode vmxerr;
    try {
        if (CheckCANStatus(vmxerr)) {
            ROS_INFO("VMXPi opened");
            can->SetMode(VMXCAN::VMXCAN_NORMAL, &vmxerr);
            can->FlushTxFIFO(&vmxerr);
            // Open receive stream for CAN Blackboard, which ensures latest packet is received
            if (!can->OpenReceiveStream(canrxhandles[0], 0x0, 0x0, 100, &vmxerr)) {
                ROS_WARN("Error opening CAN RX Stream, handle: %d\n", canrxhandles[0]);
                DisplayVMXError(vmxerr);
            } else {
                ROS_INFO("Opened CAN RX Stream, handle: %d\n", canrxhandles[0]);
                if(can->EnableReceiveStreamBlackboard(canrxhandles[0], true, &vmxerr)) {
                    ROS_INFO("Enable Blackboard on CAN RX Stream, handle: %d\n", canrxhandles[0]);
                } else {
                    ROS_WARN("Error Enabling Blackboard on CAN RX Stream, handle; %d,\n", canrxhandles[0]);
                    DisplayVMXError(vmxerr);
                }
            }
            if (!CheckCANStatus(vmxerr))
                ROS_WARN("CAN ERROR");
        }
    }
    catch (const std::exception& ex) {
        ROS_INFO("Caught exception: %s\n", ex.what());
    }
    // Start separate run loop thread to prevent blocking sent packets
    runth = std::thread(&TitanDriver::Run, this);
}

// Configure specified motor's frequency
void TitanDriver::ConfigureMotor(uint8_t motor) {
    VMXCANMessage msg, status;
    VMXErrorCode vmxerr;
    uint8_t data[8] = {
        motor, 
        (uint8_t)(frequency & 0xff), // Set MSB (most significant byte) first
        (uint8_t)(frequency>>8),
        0,0,0,0,0};
    msg.messageID = TitanMessageID::CONFIGURE_MOTOR + deviceID; 
    msg.dataSize = 8;
    msg.setData(data, 8);
    if (!can->SendMessage(msg, 0, &vmxerr))
        DisplayVMXError(vmxerr);
}

// Set specified motor's speed 
void TitanDriver::SetMotorSpeed(double speed_set, int motor) {
    // Save current speed of motor in array
    m_speed[motor] = speed_set;
    speed_set *= 100;
    uint8_t INA = 0, INB = 0;
    if (speed_set == 0) {
      INA = 1;
      INB = 1;
    } else if (speed_set > 0) {
      INA = 1;
      INB = 0;
    } else if (speed_set < 0) {
      INA = 0;
      INB = 1;
    } else {
      INA = 0;
      INB = 0;
    }
    
    VMXCANMessage msg, status;
    VMXErrorCode vmxerr;
    uint8_t data[8] = {
        (uint8_t)motor, 
        (uint8_t)abs(speed_set),
        INA, INB,
        0,0,0,0};
    msg.messageID = TitanMessageID::SET_MOTOR_SPEED + deviceID;
    msg.dataSize = 8;
    msg.setData(data, 8);
    if (!can->SendMessage(msg, 0, &vmxerr))
        DisplayVMXError(vmxerr);
}

// Stop specified motor
void TitanDriver::StopMotor(uint8_t motor) {
    uint8_t INA = 1, INB = 1; 
    VMXCANMessage msg, status;
    VMXErrorCode vmxerr;
    uint8_t data[8] = {motor,
        0,INA,INB,0,0,0,0};
    msg.messageID = TitanMessageID::SET_MOTOR_SPEED + deviceID;
    msg.dataSize = 8;
    msg.setData(data, 8);
    if (!can->SendMessage(msg, 0, &vmxerr))
        DisplayVMXError(vmxerr);
}

// Stop all motors 
void TitanDriver::StopAllMotors() {
    uint8_t INA = 1, INB = 1; 
    VMXCANMessage msg, status;
    VMXErrorCode vmxerr;
    uint8_t data[8] = {0,0,INA,INB,0,0,0,0};
    msg.messageID = TitanMessageID::SET_MOTOR_SPEED + deviceID;
    msg.dataSize = 8;
    for (int i = 0; i < 4; i++) {
        data[0] = i;
        msg.setData(data, 8);
        if (!can->SendMessage(msg, 0, &vmxerr))
            DisplayVMXError(vmxerr);
    }
}

// Get Titan's firmware and hardware information 
void TitanDriver::GetInfo(vmxpi_ros::TitanInfo *res_msg) {
    VMXErrorCode vmxerr;
    if (!CheckCANStatus(vmxerr)) 
        ROS_WARN("CAN ERROR");

    VMXCANTimestampedMessage stream_msg;
    static uint32_t msg_count;
    static uint64_t timestamp;
    ReadCANBlackboard(canrxhandles[0], TitanMessageID::RETURN_TITAN_INFO + deviceID, &stream_msg, msg_count, timestamp, vmxerr);
    
    res_msg->deviceID = stream_msg.data[0];
    // (Major.Min.Build)
    res_msg->verMaj = stream_msg.data[1]; // Major version 
    res_msg->verMin = stream_msg.data[2]; // Minor version
    res_msg->verBuild = stream_msg.data[3]; // Build version
    res_msg->hardware = stream_msg.data[4]; // Hardware (1 = Quad, 2 = Small)
    res_msg->hardwareRev = stream_msg.data[5]; // Hardware Rev
    
}

// Get all motors' frequency
void TitanDriver::GetMotorFreq(std_msgs::UInt16 res_msg[]) {
    VMXErrorCode vmxerr;
    if (!CheckCANStatus(vmxerr))
        ROS_WARN("CAN ERROR");

    VMXCANTimestampedMessage stream_msg;
    static uint32_t msg_count;
    static uint64_t timestamp;
    for (int i = 0; i < 4; i++) {
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::RETURN_MOTOR_FREQUENCY + deviceID, &stream_msg, msg_count, timestamp, vmxerr);
        res_msg[i].data = stream_msg.data[0] + (stream_msg.data[1] << 8);
    }
}

// Get Titan's Unique ID
void TitanDriver::GetUniqueID(vmxpi_ros::UniqueID *res_msg) {
    VMXErrorCode vmxerr;
    if (!CheckCANStatus(vmxerr))
        ROS_WARN("CAN ERROR");
    
    VMXCANTimestampedMessage msg1, msg2, msg3;
    static uint32_t msg_count;
    static uint64_t timestamp;
    ReadCANBlackboard(canrxhandles[0], TitanMessageID::RETURN_WORD_1 + deviceID, &msg1, msg_count, timestamp, vmxerr);
    ReadCANBlackboard(canrxhandles[0], TitanMessageID::RETURN_WORD_2 + deviceID, &msg2, msg_count, timestamp, vmxerr);
    ReadCANBlackboard(canrxhandles[0], TitanMessageID::RETURN_WORD_3 + deviceID, &msg3, msg_count, timestamp, vmxerr);
    
    // Convert read packet data for each word into hex
    stringstream ss1, ss2, ss3;
    int w1 = (msg1.data[0] + (msg1.data[1] << 8) + (msg1.data[2] << 16) + (msg1.data[3] << 24));
    int w2 = (msg2.data[0] + (msg2.data[1] << 8) + (msg2.data[2] << 16) + (msg2.data[3] << 24));
    int w3 = (msg3.data[0] + (msg3.data[1] << 8) + (msg3.data[2] << 16) + (msg3.data[3] << 24));
    ss1 << std::setfill('0') << std::setw(8) << std::hex << std::uppercase << w1;
    res_msg->word1 = ss1.str();    
    ss2 << std::setfill('0') << std::setw(8) << std::hex << std::uppercase << w2;
    res_msg->word2 = ss2.str();
    ss3 << std::setfill('0') << std::setw(8) << std::hex << std::uppercase << w3;
    res_msg->word3 = ss3.str();
    
    // Unique ID format is (word1.word2.word3)
}

// Reset encoder count for specified motor
void TitanDriver::ResetEncoder(uint8_t motor) {
    VMXCANMessage msg;
    VMXErrorCode vmxerr;
    uint8_t data[8] = {motor,0,0,0,0,0,0,0};
    msg.messageID = TitanMessageID::RESET_ENCODER + deviceID;
    msg.dataSize = 8;
    msg.setData(data, 8);
    if (!can->SendMessage(msg, 0, &vmxerr))
        DisplayVMXError(vmxerr);
}

// Reset all encoder counts
void TitanDriver::ResetAllEncoders() {
    VMXCANMessage msg;
    VMXErrorCode vmxerr;
    uint8_t data[8] = {0,0,0,0,0,0,0,0};
    msg.messageID = TitanMessageID::RESET_ENCODER + deviceID;
    msg.dataSize = 8;
    for (int i = 0; i < 4; i++) {
        data[0] = i;
        msg.setData(data, 8);
        if (!can->SendMessage(msg, 0, &vmxerr))
            DisplayVMXError(vmxerr);
    }
}

// Set the stop mode on the specified motor (0 = coast, 1 = break)
void TitanDriver::SetMotorStopMode(uint8_t motor, int8_t mode) {
    VMXCANMessage msg;
    VMXErrorCode vmxerr;
    uint8_t data[8] = {motor,(uint8_t)mode,0,0,0,0,0,0};
    msg.messageID = TitanMessageID::SET_MOTOR_STOP_MODE + deviceID;
    msg.dataSize = 8;
    if (!can->SendMessage(msg, 0, &vmxerr))
        DisplayVMXError(vmxerr);
}

/* Read the encoder count + RPM packet for each encoder and saving it
 * Byte 0-3 is encoder count, byte 4-5 is RPM (only for 1.2 Titan Firmware)
 * Byte 0-3 is encoder count of the encoder flag (for 1.0 Titan Firmware)
 * Byte 0-1 is the rpm of the motor (for 1.0 Titan Firmware) */
void TitanDriver::GetCountRPM(int8_t motor) {
    VMXErrorCode vmxerr;
    if (!CheckCANStatus(vmxerr))
        ROS_WARN("CAN ERROR");
    
    VMXCANTimestampedMessage msg;
    VMXCANTimestampedMessage msg2;
    static uint64_t timestamp = 0;
    static uint32_t msg_count;
    if (motor == 0) {
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::ENCODER_0_OUTPUT + deviceID, &msg, msg_count, timestamp, vmxerr);
        enc_count[motor] = msg.data[0] + (msg.data[1] << 8) + (msg.data[2] << 16) + (msg.data[3] << 24);
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::RPM_0_OUTPUT + deviceID, &msg2, msg_count, timestamp, vmxerr);
        rpm[motor] = msg2.data[0] + (msg2.data[1] << 8);
    } else if (motor == 1) {
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::ENCODER_1_OUTPUT + deviceID, &msg, msg_count, timestamp, vmxerr);
        enc_count[motor] = msg.data[0] + (msg.data[1] << 8) + (msg.data[2] << 16) + (msg.data[3] << 24);
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::RPM_1_OUTPUT + deviceID, &msg2, msg_count, timestamp, vmxerr);
        rpm[motor] = msg2.data[0] + (msg2.data[1] << 8);
    } else if (motor == 2) {
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::ENCODER_2_OUTPUT + deviceID, &msg, msg_count, timestamp, vmxerr);
        enc_count[motor] = msg.data[0] + (msg.data[1] << 8) + (msg.data[2] << 16) + (msg.data[3] << 24);
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::RPM_2_OUTPUT + deviceID, &msg2, msg_count, timestamp, vmxerr);
        rpm[motor] = msg2.data[0] + (msg2.data[1] << 8);
    } else if (motor == 3) {
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::ENCODER_3_OUTPUT+ deviceID, &msg, msg_count, timestamp, vmxerr);
        enc_count[motor] = msg.data[0] + (msg.data[1] << 8) + (msg.data[2] << 16) + (msg.data[3] << 24);
        ReadCANBlackboard(canrxhandles[0], TitanMessageID::RPM_3_OUTPUT + deviceID, &msg2, msg_count, timestamp, vmxerr);
        rpm[motor] = msg2.data[0] + (msg2.data[1] << 8);
    }
}

/* Get the count of the specified encoder, note this
 * is not the encoder distance */
void TitanDriver::GetEncoderCount(std_msgs::Int32 *res_msg, int8_t encoder) {  
    res_msg->data = enc_count[encoder];
}

/* Configure the encoder's distance per tick
 * For more info on this: https://docs.wsr.studica.com/en/latest/docs/Sensors/encoders.html */
void TitanDriver::ConfigureEncoder(int8_t encoder, double radius, double encoderPPR, double gearRatio) {
    double encoderPR = 0;
    if(encoder == 0){
        encoderPR = encoderPPR * gearRatio;
        ENCODER_0_DISTANCE_PER_TICK = (M_PI * 2 * radius) / encoderPPR;
    } else if (encoder == 1){
        encoderPR = encoderPPR * gearRatio;
        ENCODER_1_DISTANCE_PER_TICK = (M_PI * 2 * radius) / encoderPPR;
    } else if (encoder == 2){
        encoderPR = encoderPPR * gearRatio;
        ENCODER_2_DISTANCE_PER_TICK = (M_PI * 2 * radius) / encoderPPR;
    } else if (encoder == 3){
        encoderPR = encoderPPR * gearRatio;
        ENCODER_3_DISTANCE_PER_TICK = (M_PI * 2 * radius) / encoderPPR;
    }
}

// Calculate the encoder's distance based on its current count
void TitanDriver::GetEncoderDist(std_msgs::Float32 *res_msg, int8_t encoder) {
    if (encoder == 0) 
        res_msg->data = (float)enc_count[encoder] * ENCODER_0_DISTANCE_PER_TICK;
    else if (encoder == 1)
        res_msg->data = (float)enc_count[encoder] * ENCODER_1_DISTANCE_PER_TICK;
    else if (encoder == 2)
        res_msg->data = (float)enc_count[encoder] * ENCODER_2_DISTANCE_PER_TICK;
    else if (encoder == 3)
        res_msg->data = (float)enc_count[encoder] * ENCODER_3_DISTANCE_PER_TICK;
}

// Get the RPM of the specified encoder
void TitanDriver::GetRPM(std_msgs::Int16 *res_msg, uint8_t motor) {
    res_msg->data = rpm[motor];
}

// Get the limit switch values for each motor as a boolean value
void TitanDriver::GetLimitSwitch(vmxpi_ros::LimitSwitch *res_msg) {
    
    VMXErrorCode vmxerr;
    if (!CheckCANStatus(vmxerr))
        ROS_WARN("CAN ERROR");
    
    VMXCANTimestampedMessage msg;
    static uint32_t msg_count;
    static uint64_t timestamp;
    ReadCANBlackboard(canrxhandles[0], TitanMessageID::LIMIT_SWITCH_OUTPUT + deviceID, &msg, msg_count, timestamp, vmxerr);
    
    res_msg->limitswitch.push_back(msg.data[0]);
    res_msg->limitswitch.push_back(msg.data[1]);
    res_msg->limitswitch.push_back(msg.data[2]);
    res_msg->limitswitch.push_back(msg.data[3]);
    res_msg->limitswitch.push_back(msg.data[4]);
    res_msg->limitswitch.push_back(msg.data[5]);
    res_msg->limitswitch.push_back(msg.data[6]);
    res_msg->limitswitch.push_back(msg.data[7]);
}

// Get the current MCU (CPU) temperature
void TitanDriver::GetMCUTemp(std_msgs::Float32 *res_msg) {
    VMXErrorCode vmxerr;
    if (!CheckCANStatus(vmxerr))
        ROS_WARN("CAN ERROR");
    
    VMXCANTimestampedMessage msg;
    static uint32_t msg_count;
    static uint64_t timestamp;
    ReadCANBlackboard(canrxhandles[0], TitanMessageID::MCU_TEMP + deviceID, &msg, msg_count, timestamp, vmxerr);
    
    // First byte is integer, second byte is the decimal value (msg.data[0]).(msg.data[1])
    res_msg->data = (float)msg.data[0] + ((float)msg.data[1] / 100);
}

// Get specified motor's speed
void TitanDriver::GetSpeed(std_msgs::Float32 *res_msg, uint8_t motor) { res_msg->data = m_speed[motor]; }

// Sets the current flag being sent by the run loop to enabled
void TitanDriver::Enable() { enabled = true; }

// Sets the current flag being sent by the run loop to disabled
void TitanDriver::Disable() { enabled = false; }

TitanDriver::~TitanDriver() { runth.detach(); }

// Blackboard read function utilizing VMXPi's blackboard functionality
void TitanDriver::ReadCANBlackboard(VMXCANReceiveStreamHandle handle, int msgID, VMXCANTimestampedMessage* data, uint32_t length, uint64_t timestamp, VMXErrorCode vmxerr)
{
    VMXCANTimestampedMessage blackboard_msg;
	  bool already_retrieved;
    uint64_t sys_timestamp;
    // If the latest packet from the specified message ID changes, copy new data to reference packet
    if(can->GetBlackboardEntry(handle, msgID, blackboard_msg, sys_timestamp, already_retrieved, &vmxerr))
    {
        length = blackboard_msg.dataSize;
        timestamp = blackboard_msg.timeStampMS;
        for(int k = 0; k < blackboard_msg.dataSize; k++)
        {
            data->data[k] = blackboard_msg.data[k];
        }
    } else {
      DisplayVMXError(vmxerr);
    }
}

