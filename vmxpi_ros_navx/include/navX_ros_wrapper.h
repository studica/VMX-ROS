#pragma once

#include "VMXPi.h"
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <sys/syscall.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <vmxpi_ros/StringRes.h>   
#include <vmxpi_ros/IntRes.h> 

class navXROSWrapper {
    private:
        vmx::AHRS *ahrs;
        ros::NodeHandle n;
        ros::Publisher rpy_pub, pitch_pub, roll_pub, yaw_pub, angle_pub, heading_pub, linaccel_pub, fusedheading_pub,
                timestamp_pub, quaternion_pub, velocity_pub, displace_pub,
                linaccelx_pub, linaccely_pub, linaccelz_pub,
                quatx_pub, quaty_pub, quatz_pub, quatw_pub,
                velx_pub, vely_pub, velz_pub,
                displacex_pub, displacey_pub, displacez_pub,
                rawgyro_pub, rawgyrox_pub, rawgyroy_pub, rawgyroz_pub,
                rawaccel_pub, rawaccelx_pub, rawaccely_pub, rawaccelz_pub,
                rawmag_pub, rawmagx_pub, rawmagy_pub, rawmagz_pub,
                pose_pub;
        ros::ServiceServer firmware_ver, reset_navx, reset_displacement, update_rate;
        
        std::thread runth;
        
        
    public:
        navXROSWrapper(ros::NodeHandle *nh, VMXPi *vmx);
        ~navXROSWrapper();
        
        // Get current firmware version of navX
        bool GetFirmwareVersion(vmxpi_ros::StringRes::Request &req, vmxpi_ros::StringRes::Response &res);
        
        // Grouped publisher for pitch, roll, and yaw (-180 to 180 deg)
        void PubRollPitchYaw();
        
        void PubPitch();
        void PubRoll();
        void PubYaw();
        void PubAngle();
        
        // Publish compass heading
        void PubHeading();
        
        // Grouped publisher for linear acceleration (G)
        void PubLinearAccel();
        
        void PubLinAccelX();
        void PubLinAccelY();
        void PubLinAccelZ();
        
        // Publish fused (9-axis) heading, magnetometer calibration is required
        void PubFusedHeading();
        
        // Publish timestamp of last sensor message
        void PubTimestamp();
        
        // Grouped publisher for quaternion axis w.r.t where yaw was last reset
        void PubQuaternion();
        
        void PubQuatX();
        void PubQuatY();
        void PubQuatZ();
        void PubQuatW();
        
        // Grouped publisher for velocity (m/s) [experimental]
        void PubVelocity();
        
        void PubVelX();
        void PubVelY();
        void PubVelZ();
        
        // Grouped publisher for displacement from position when reset was invoked (m)
        void PubDisplacement();
        
        void PubDisplaceX();
        void PubDisplaceY();
        void PubDisplaceZ();
        
        // Grouped publisher for raw gyro rotation rate (deg/s)
        void PubRawGyro();
        
        void PubRawGyroX();
        void PubRawGyroY();
        void PubRawGyroZ();
        
        // Grouped publisher for raw acceleration (G)
        void PubRawAccel();
        
        void PubRawAccelX();
        void PubRawAccelY();
        void PubRawAccelZ();
        
        // Grouped publisher for raw magnetometer reading (uTesla)
        void PubRawMag();
        
        void PubRawMagX();
        void PubRawMagY();
        void PubRawMagZ();
        
        // Publish pose
        void PubPose();
        
        // Get navX's update rate
        bool GetUpdateRate(vmxpi_ros::IntRes::Request &req, vmxpi_ros::IntRes::Response &res);
        
        // Reset yaw for quaternion
        bool Reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        
        // Reset displacement, new origin will be where this is invoked 
        bool ResetDisplacement(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        
        bool IsCalibrating();

        bool IsConnected();

        bool IsMoving();
        
        bool IsRotating();

        bool IsMagnDisturbance();

        bool IsMagCalibrated();
        
        void Run_t();

};
