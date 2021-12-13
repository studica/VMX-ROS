#include "TitanDriver_ros_wrapper.h"

TitanDriverROSWrapper::TitanDriverROSWrapper(ros::NodeHandle *nh, VMXPi *vmx) :
    can{&vmx->getCAN()} {
    // If new wrapper is initialized, delete previous one
    Titan.reset(new TitanDriver(42, 15600, can));

    // Advertise topics and service servers
    motor_speed = nh->advertiseService(
        "titan/set_motor_speed", &TitanDriverROSWrapper::SetMotorSpeed, this);
            
    stop_motor_server = nh->advertiseService(
        "titan/stop_motors", &TitanDriverROSWrapper::callbackStop, this);

    enable_server = nh->advertiseService(
        "titan/enable", &TitanDriverROSWrapper::Enable, this);
                
    disable_server = nh->advertiseService(
        "titan/disable", &TitanDriverROSWrapper::Disable, this);
            
    reset_server = nh->advertiseService(
        "titan/reset_encoder", &TitanDriverROSWrapper::ResetAllEncoders, this);
            
    motor_stop_mode_server = nh->advertiseService(
        "titan/motor_stopmode", &TitanDriverROSWrapper::SetMotorStopMode, this);

    uniqueid_pub = nh->advertise<vmxpi_ros::UniqueID>("titan/unique_id", 1);

    info_publisher = nh->advertise<vmxpi_ros::TitanInfo>("titan/info", 1);

    motor0freq_pub = nh->advertise<std_msgs::UInt16>("titan/motor0/freq", 1);
    motor1freq_pub = nh->advertise<std_msgs::UInt16>("titan/motor1/freq", 1);
    motor2freq_pub = nh->advertise<std_msgs::UInt16>("titan/motor2/freq", 1);
    motor3freq_pub = nh->advertise<std_msgs::UInt16>("titan/motor3/freq", 1);
    
    motor0speed_pub = nh->advertise<std_msgs::Float32>("titan/motor0/speed", 1);
    motor1speed_pub = nh->advertise<std_msgs::Float32>("titan/motor1/speed", 1);
    motor2speed_pub = nh->advertise<std_msgs::Float32>("titan/motor2/speed", 1);
    motor3speed_pub = nh->advertise<std_msgs::Float32>("titan/motor3/speed", 1);

    enc0_pub = nh->advertise<std_msgs::Int32>("titan/encoder0/count", 1);
    enc1_pub = nh->advertise<std_msgs::Int32>("titan/encoder1/count", 1);
    enc2_pub = nh->advertise<std_msgs::Int32>("titan/encoder2/count", 1);
    enc3_pub = nh->advertise<std_msgs::Int32>("titan/encoder3/count", 1);
    
    encdist0_pub = nh->advertise<std_msgs::Float32>("titan/encoder0/distance", 1);
    encdist1_pub = nh->advertise<std_msgs::Float32>("titan/encoder1/distance", 1);
    encdist2_pub = nh->advertise<std_msgs::Float32>("titan/encoder2/distance", 1);
    encdist3_pub = nh->advertise<std_msgs::Float32>("titan/encoder3/distance", 1);
            
    rpm0_pub = nh->advertise<std_msgs::Int16>("titan/motor0/rpm", 1);
    rpm1_pub = nh->advertise<std_msgs::Int16>("titan/motor1/rpm", 1);
    rpm2_pub = nh->advertise<std_msgs::Int16>("titan/motor2/rpm", 1);
    rpm3_pub = nh->advertise<std_msgs::Int16>("titan/motor3/rpm", 1);

    limitswitch_pub = nh->advertise<vmxpi_ros::LimitSwitch>("titan/limit_switch", 1);
    mcutemp_pub = nh->advertise<std_msgs::Float32>("titan/mcu_temp", 1);
    
    run1th = std::thread(&TitanDriverROSWrapper::Run1_t, this);
    run2th = std::thread(&TitanDriverROSWrapper::Run2_t, this);
}

TitanDriverROSWrapper::~TitanDriverROSWrapper() {
    run1th.join();
    run2th.join();
    StopAllMotors();
}

bool TitanDriverROSWrapper::SetMotorSpeed(vmxpi_ros::MotorSpeed::Request &req, vmxpi_ros::MotorSpeed::Response &res) {
    Titan->SetMotorSpeed((double)req.speed, req.motor);
    return true;
}

bool TitanDriverROSWrapper::callbackStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    Titan->StopAllMotors();
    res.success = true;
    res.message = "Stopping Motors.";
    return true;
}

void TitanDriverROSWrapper::StopMotor(int motor) { Titan->StopMotor(motor); }

void TitanDriverROSWrapper::StopAllMotors() { Titan->StopAllMotors(); }

bool TitanDriverROSWrapper::Enable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    Titan->Enable();
    res.success = true;
    res.message = "Enabling.";
    return true;
}

bool TitanDriverROSWrapper::ResetAllEncoders(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    Titan->ResetAllEncoders();
    res.success = true;
    res.message = "Reseting Encoder counts.";
    return true;
}

bool TitanDriverROSWrapper::Disable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    Titan->Disable();
    res.success = true;
    res.message = "Disabling.";
    return true;
}
    
bool TitanDriverROSWrapper::ResetEncoder(vmxpi_ros::Int::Request &req, vmxpi_ros::Int::Response &res) {
    Titan->ResetEncoder(req.data);
    return true;
}

bool TitanDriverROSWrapper::SetMotorStopMode(vmxpi_ros::StopMode::Request &req, vmxpi_ros::StopMode::Response &res) {
    Titan->SetMotorStopMode(req.motor, req.mode);
    return true;
}

void TitanDriverROSWrapper::PubUniqueID() {
    vmxpi_ros::UniqueID msg;
    Titan->GetUniqueID(&msg);
    uniqueid_pub.publish(msg);
}

void TitanDriverROSWrapper::PubTitanInfo() {
    vmxpi_ros::TitanInfo msg;
    Titan->GetInfo(&msg);
    info_publisher.publish(msg);
}

void TitanDriverROSWrapper::PubMotorSpeed(uint8_t motor) {
    std_msgs::Float32 msg;
    Titan->GetSpeed(&msg, motor);
    if (motor == 0)
        motor0speed_pub.publish(msg);
    else if (motor == 1)
        motor1speed_pub.publish(msg);
    else if (motor == 2)
        motor2speed_pub.publish(msg);
    else if (motor == 3)
        motor3speed_pub.publish(msg);
}

void TitanDriverROSWrapper::PubMotorFreq() {
    std_msgs::UInt16 msg[4];
    Titan->GetMotorFreq(msg);
    motor0freq_pub.publish(msg[0]);
    motor1freq_pub.publish(msg[1]);
    motor2freq_pub.publish(msg[2]);
    motor3freq_pub.publish(msg[3]);
}

void TitanDriverROSWrapper::GetCountRPM() {
    Titan->GetCountRPM(0);
    Titan->GetCountRPM(1);
    Titan->GetCountRPM(2);
    Titan->GetCountRPM(3);
}

void TitanDriverROSWrapper::PubEncoderCount(int8_t encoder) {
    std_msgs::Int32 msg;
    Titan->GetEncoderCount(&msg, encoder);
    if (encoder == 0) 
        enc0_pub.publish(msg);
    else if (encoder == 1)
        enc1_pub.publish(msg);
    else if (encoder == 2)
        enc2_pub.publish(msg);
    else if (encoder == 3)
        enc3_pub.publish(msg);
}

void TitanDriverROSWrapper::PubEncoderDist(int8_t encoder) {
    std_msgs::Float32 msg;
    Titan->GetEncoderDist(&msg, encoder);
    if (encoder == 0) 
        encdist0_pub.publish(msg);
    else if (encoder == 1) 
        encdist1_pub.publish(msg);
    else if (encoder == 2) 
        encdist2_pub.publish(msg);
    else if (encoder == 3) 
        encdist3_pub.publish(msg);
}

void TitanDriverROSWrapper::PubRPM(uint8_t motor) {
    std_msgs::Int16 msg;
    Titan->GetRPM(&msg, motor);
    if (motor == 0) 
        rpm0_pub.publish(msg);
    else if (motor == 1)
        rpm1_pub.publish(msg);
    else if (motor == 2) 
        rpm2_pub.publish(msg);
    else if (motor == 3) 
        rpm3_pub.publish(msg);
}

void TitanDriverROSWrapper::PubLimitSwitch() {
    vmxpi_ros::LimitSwitch msg;
    Titan->GetLimitSwitch(&msg);
    limitswitch_pub.publish(msg);
}

void TitanDriverROSWrapper::PubMCUTemp() {
    std_msgs::Float32 msg;
    Titan->GetMCUTemp(&msg);
    mcutemp_pub.publish(msg);
}

void TitanDriverROSWrapper::Run1_t() {
    ros::Rate r(1);
    ROS_INFO_STREAM("Titan pub thread 1: " << syscall(SYS_gettid));
    while (ros::ok()) {
      
      PubMotorFreq();
      PubUniqueID();
      PubTitanInfo();
        
      PubLimitSwitch();
      PubMCUTemp();
      r.sleep();
    }
}

void TitanDriverROSWrapper::Run2_t() {
    ros::Rate r(100);
    ROS_INFO_STREAM("Titan pub thread 2: " << syscall(SYS_gettid));
    while (ros::ok()) {
      GetCountRPM();
      PubEncoderCount(0);
      PubEncoderCount(1);
      PubEncoderCount(2);
      PubEncoderCount(3);
        
      PubMotorSpeed(0);
      PubMotorSpeed(1);
      PubMotorSpeed(2);
      PubMotorSpeed(3);
      
      PubEncoderDist(0);
      PubEncoderDist(1);
      PubEncoderDist(2);
      PubEncoderDist(3); 
    
      PubRPM(0);
      PubRPM(1);
      PubRPM(2);
      PubRPM(3);
      r.sleep();
    }
}


