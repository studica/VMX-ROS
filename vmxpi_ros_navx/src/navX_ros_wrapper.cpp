#include "navX_ros_wrapper.h"

navXROSWrapper::navXROSWrapper(ros::NodeHandle *nh, VMXPi *vmx) :
    ahrs{&vmx->getAHRS()} {
    
    // Advertise topics and service servers
    rpy_pub = nh->advertise<geometry_msgs::Vector3>("navx/roll_pitch_yaw", 1);
    
    pitch_pub = nh->advertise<std_msgs::Float32>("navx/pitch", 1);
    roll_pub = nh->advertise<std_msgs::Float32>("navx/roll", 1);
    yaw_pub = nh->advertise<std_msgs::Float32>("navx/yaw", 1);
    angle_pub = nh->advertise<std_msgs::Float32>("navx/angle", 1);
    
    heading_pub = nh->advertise<std_msgs::Float32>("navx/heading", 1);
    linaccel_pub = nh->advertise<geometry_msgs::Vector3>("navx/linear_accel", 1);
    
    linaccelx_pub = nh->advertise<std_msgs::Float32>("navx/linear_accel/x", 1);
    linaccely_pub = nh->advertise<std_msgs::Float32>("navx/linear_accel/y", 1);
    linaccelz_pub = nh->advertise<std_msgs::Float32>("navx/linear_accel/z", 1);
    
    fusedheading_pub = nh->advertise<std_msgs::Float32>("navx/fused_heading", 1);
    timestamp_pub = nh->advertise<std_msgs::UInt64>("navx/last_sensor_timestamp", 1);
    
    quaternion_pub = nh->advertise<geometry_msgs::Quaternion>("navx/quaternion", 1);
    
    quatx_pub = nh->advertise<std_msgs::Float64>("navx/quaternion/x", 1);
    quaty_pub = nh->advertise<std_msgs::Float64>("navx/quaternion/y", 1);
    quatz_pub = nh->advertise<std_msgs::Float64>("navx/quaternion/z", 1);
    quatw_pub = nh->advertise<std_msgs::Float64>("navx/quaternion/w", 1);
    
    velocity_pub = nh->advertise<geometry_msgs::Vector3>("navx/velocity", 1);
    
    velx_pub = nh->advertise<std_msgs::Float32>("navx/velocity/x", 1);
    vely_pub = nh->advertise<std_msgs::Float32>("navx/velocity/y", 1);
    velz_pub = nh->advertise<std_msgs::Float32>("navx/velocity/z", 1);
    
    displace_pub = nh->advertise<geometry_msgs::Point>("navx/displacement", 1);
    
    displacex_pub = nh->advertise<std_msgs::Float32>("navx/displacement/x", 1);
    displacey_pub = nh->advertise<std_msgs::Float32>("navx/displacement/y", 1);
    displacez_pub = nh->advertise<std_msgs::Float32>("navx/displacement/z", 1);
    
    rawgyro_pub = nh->advertise<geometry_msgs::Vector3>("navx/raw_gyro", 1);
    
    rawgyrox_pub = nh->advertise<std_msgs::Float32>("navx/raw_gyro/x", 1);
    rawgyroy_pub = nh->advertise<std_msgs::Float32>("navx/raw_gyro/y", 1);
    rawgyroz_pub = nh->advertise<std_msgs::Float32>("navx/raw_gyro/z", 1);
    
    rawaccel_pub = nh->advertise<geometry_msgs::Vector3>("navx/raw_accel", 1);
    
    rawaccelx_pub = nh->advertise<std_msgs::Float32>("navx/raw_accel/x", 1);
    rawaccely_pub = nh->advertise<std_msgs::Float32>("navx/raw_accel/y", 1);
    rawaccelz_pub = nh->advertise<std_msgs::Float32>("navx/raw_accel/z", 1);
    
    rawmag_pub = nh->advertise<geometry_msgs::Vector3>("navx/raw_mag", 1);
    
    rawmagx_pub = nh->advertise<std_msgs::Float32>("navx/raw_mag/x", 1);
    rawmagy_pub = nh->advertise<std_msgs::Float32>("navx/raw_mag/y", 1);
    rawmagz_pub = nh->advertise<std_msgs::Float32>("navx/raw_mag/z", 1);
    
    pose_pub = nh->advertise<geometry_msgs::Pose>("navx/pose", 1);
    
    firmware_ver = nh->advertiseService(
        "get_ahrs_firmware_version", &navXROSWrapper::GetFirmwareVersion, this);
    
    reset_navx = nh->advertiseService(
        "reset_navx", &navXROSWrapper::Reset, this);
        
    reset_displacement = nh->advertiseService(
        "reset_displacement", &navXROSWrapper::ResetDisplacement, this);
    
    update_rate = nh->advertiseService(
        "get_update_rate", &navXROSWrapper::GetUpdateRate, this);
        
    runth = std::thread(&navXROSWrapper::Run_t, this);
}

navXROSWrapper::~navXROSWrapper() {
    runth.join();
}

// Get current firmware version of navX
bool navXROSWrapper::GetFirmwareVersion(vmxpi_ros::StringRes::Request &req, vmxpi_ros::StringRes::Response &res) {
    res.data = ahrs->GetFirmwareVersion();
    return true;
}

// Grouped publisher for pitch, roll, and yaw (-180 to 180 deg)
void navXROSWrapper::PubRollPitchYaw() {
    geometry_msgs::Vector3 msg;
    msg.x = ahrs->GetRoll();
    msg.y = ahrs->GetPitch();
    msg.z = ahrs->GetYaw();
    rpy_pub.publish(msg);
}

// Publish pitch
void navXROSWrapper::PubPitch() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetPitch();
    pitch_pub.publish(msg);
}

// Publish roll
void navXROSWrapper::PubRoll() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRoll();
    roll_pub.publish(msg);
}

// Publish yaw
void navXROSWrapper::PubYaw() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetYaw();
    yaw_pub.publish(msg);
}

//Publish angle
void navXROSWrapper::PubAngle() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetAngle();
    angle_pub.publish(msg);
}

// Publish compass heading
void navXROSWrapper::PubHeading() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetCompassHeading();
    heading_pub.publish(msg);
}

// Grouped publisher for linear acceleration (G)
void navXROSWrapper::PubLinearAccel() {
    geometry_msgs::Vector3 msg;
    msg.x = ahrs->GetWorldLinearAccelX();
    msg.y = ahrs->GetWorldLinearAccelY();
    msg.z = ahrs->GetWorldLinearAccelZ();
    linaccel_pub.publish(msg);
}

// Publish x-axis linear acceleration
void navXROSWrapper::PubLinAccelX() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetWorldLinearAccelX();
    linaccelx_pub.publish(msg);
}

// Publish y-axis linear acceleration
void navXROSWrapper::PubLinAccelY() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetWorldLinearAccelY();
    linaccely_pub.publish(msg);
}

// Publish z-axis linear acceleration
void navXROSWrapper::PubLinAccelZ() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetWorldLinearAccelZ();
    linaccelz_pub.publish(msg);
}

// Publish fused (9-axis) heading, magnetometer calibration is required
void navXROSWrapper::PubFusedHeading() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetFusedHeading();
    fusedheading_pub.publish(msg);
}

// Publish timestamp of last sensor message
void navXROSWrapper::PubTimestamp() {
    std_msgs::UInt64 msg;
    msg.data = ahrs->GetLastSensorTimestamp();
    timestamp_pub.publish(msg);
}

// Grouped publisher for quaternion axis w.r.t where yaw was last reset
void navXROSWrapper::PubQuaternion() {
    geometry_msgs::Quaternion msg;
    msg.x = ahrs->GetQuaternionX();
    msg.y = ahrs->GetQuaternionY();
    msg.z = ahrs->GetQuaternionZ();
    msg.w = ahrs->GetQuaternionW();
    quaternion_pub.publish(msg);
}

// Publish real portion of x-axis quaternion
void navXROSWrapper::PubQuatX() {
    std_msgs::Float64 msg;
    msg.data = ahrs->GetQuaternionX();
    quatx_pub.publish(msg);
}

// Publish real portion of y-axis quaternion
void navXROSWrapper::PubQuatY() {
    std_msgs::Float64 msg;
    msg.data = ahrs->GetQuaternionY();
    quaty_pub.publish(msg);
}

// Publish real portion of z-axis quaternion
void navXROSWrapper::PubQuatZ() {
    std_msgs::Float64 msg;
    msg.data = ahrs->GetQuaternionZ();
    quatz_pub.publish(msg);
}

// Publish imaginary portion of quaternion
void navXROSWrapper::PubQuatW() {
    std_msgs::Float64 msg;
    msg.data = ahrs->GetQuaternionW();
    quatw_pub.publish(msg);
}

// Grouped publisher for velocity (m/s) [experimental]
void navXROSWrapper::PubVelocity() {
    geometry_msgs::Vector3 msg;
    msg.x = ahrs->GetVelocityX();
    msg.y = ahrs->GetVelocityY();
    msg.z = ahrs->GetVelocityZ();
    velocity_pub.publish(msg);
}

// Publish x-axis velocity
void navXROSWrapper::PubVelX() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetVelocityX();
    velx_pub.publish(msg);
}

// Publish y-axis velocity
void navXROSWrapper::PubVelY() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetVelocityY();
    vely_pub.publish(msg);
}

// Publish z-axis velocity
void navXROSWrapper::PubVelZ() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetVelocityZ();
    velz_pub.publish(msg);
}

// Grouped publisher for displacement from position when reset was invoked (m)
void navXROSWrapper::PubDisplacement() {
    geometry_msgs::Point msg;
    msg.x = ahrs->GetDisplacementX();
    msg.y = ahrs->GetDisplacementY();
    msg.z = ahrs->GetDisplacementZ();
    displace_pub.publish(msg);
}

// Publish x-axis displacement
void navXROSWrapper::PubDisplaceX() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetDisplacementX();
    displacex_pub.publish(msg);
}

// Publish y-axis displacement
void navXROSWrapper::PubDisplaceY() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetDisplacementY();
    displacey_pub.publish(msg);
}

// Publish z-axis displacement
void navXROSWrapper::PubDisplaceZ() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetDisplacementZ();
    displacez_pub.publish(msg);
}

// Grouped publisher for raw gyro rotation rate (deg/s)
void navXROSWrapper::PubRawGyro() {
    geometry_msgs::Vector3 msg;
    msg.x = ahrs->GetRawGyroX();
    msg.y = ahrs->GetRawGyroY();
    msg.z = ahrs->GetRawGyroZ();
    rawgyro_pub.publish(msg);
}

// Publish raw x-axis gyro rotation rate
void navXROSWrapper::PubRawGyroX() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawGyroX();
    rawgyrox_pub.publish(msg);
}

// Publish raw y-axis gyro rotation rate
void navXROSWrapper::PubRawGyroY() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawGyroY();
    rawgyroy_pub.publish(msg);
}

// Publish raw z-axis gyro rotation rate
void navXROSWrapper::PubRawGyroZ() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawGyroZ();
    rawgyroz_pub.publish(msg);
}

// Grouped publisher for raw acceleration (G)
void navXROSWrapper::PubRawAccel() {
    geometry_msgs::Vector3 msg;
    msg.x = ahrs->GetRawAccelX();
    msg.y = ahrs->GetRawAccelY();
    msg.z = ahrs->GetRawAccelZ();
    rawaccel_pub.publish(msg);
}

// Publish raw x-axis acceleration
void navXROSWrapper::PubRawAccelX() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawAccelX();
    rawaccelx_pub.publish(msg);
}

// Publish raw y-axis acceleration
void navXROSWrapper::PubRawAccelY() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawAccelY();
    rawaccely_pub.publish(msg);
}

// Publish raw z-axis acceleration
void navXROSWrapper::PubRawAccelZ() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawAccelZ();
    rawaccelz_pub.publish(msg);
}

// Grouped publisher for raw magnetometer reading (uTesla)
void navXROSWrapper::PubRawMag() {
    geometry_msgs::Vector3 msg;
    msg.x = ahrs->GetRawMagX();
    msg.y = ahrs->GetRawMagY();
    msg.z = ahrs->GetRawMagZ();
    rawmag_pub.publish(msg);
}

// Publish raw x-axis magnetometer reading
void navXROSWrapper::PubRawMagX() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawMagX();
    rawmagx_pub.publish(msg);
}

// Publish raw y-axis magnetometer reading
void navXROSWrapper::PubRawMagY() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawMagY();
    rawmagy_pub.publish(msg);
}

// Publish raw z-axis magnetometer reading
void navXROSWrapper::PubRawMagZ() {
    std_msgs::Float32 msg;
    msg.data = ahrs->GetRawMagZ();
    rawmagz_pub.publish(msg);
}

void navXROSWrapper::PubPose() {
    geometry_msgs::Point p_msg;
    p_msg.x = ahrs->GetDisplacementX();
    p_msg.y = ahrs->GetDisplacementY();
    p_msg.z = ahrs->GetDisplacementZ();
    
    geometry_msgs::Quaternion q_msg;
    q_msg.x = ahrs->GetQuaternionX();
    q_msg.y = ahrs->GetQuaternionY();
    q_msg.z = ahrs->GetQuaternionZ();
    q_msg.w = ahrs->GetQuaternionW();
    
    geometry_msgs::Pose msg;
    msg.position = p_msg;
    msg.orientation = q_msg;
    pose_pub.publish(msg);
}

// Get navX's update rate
bool navXROSWrapper::GetUpdateRate(vmxpi_ros::IntRes::Request &req, vmxpi_ros::IntRes::Response &res) {
    res.data = ahrs->GetActualUpdateRate();
    return true;
}

// Reset yaw for quaternion
bool navXROSWrapper::Reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ahrs->ZeroYaw(); 
    return true;
}

// Reset displacement, new origin will be where this is invoked 
bool navXROSWrapper::ResetDisplacement(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) { 
    ahrs->ResetDisplacement();
    return true;
}

bool navXROSWrapper::IsCalibrating() { return ahrs->IsCalibrating(); }

bool navXROSWrapper::IsConnected() { return ahrs->IsConnected(); }

bool navXROSWrapper::IsMoving() { return ahrs->IsMoving(); }
        
bool navXROSWrapper::IsRotating() { return ahrs->IsRotating(); }

bool navXROSWrapper::IsMagnDisturbance() { return ahrs->IsMagneticDisturbance(); }

bool navXROSWrapper::IsMagCalibrated() { return ahrs->IsMagnetometerCalibrated(); }

void navXROSWrapper::Run_t() {
    ros::Rate r(50);
    ROS_INFO_STREAM("navX pub thread: " << syscall(SYS_gettid));
    while (ros::ok()) { 
      PubRollPitchYaw();   
      PubPitch();
      PubRoll();
      PubYaw(); 
      PubAngle();
      PubHeading();
      PubLinearAccel();
      PubLinAccelX();
      PubLinAccelY();
      PubLinAccelZ();
      PubFusedHeading();
      PubQuaternion();
      PubQuatX();
      PubQuatY();
      PubQuatZ();
      PubQuatW();
      PubVelocity();
      PubVelX();
      PubVelY();
      PubVelZ();
      PubDisplacement();
      PubDisplaceX();
      PubDisplaceY();
      PubDisplaceZ();
      PubRawGyro();
      PubRawGyroX();
      PubRawGyroY();
      PubRawGyroZ();
      PubRawAccel();
      PubRawAccelX();
      PubRawAccelY();
      PubRawAccelZ();
      PubRawMag();
      PubRawMagX();
      PubRawMagY();
      PubRawMagZ();
      r.sleep();
    }
}
