#pragma once

#include <memory>
#include <ros/ros.h>
#include <stdint.h>
#include <thread>
#include "TitanDriver_ros.h"
#include <unistd.h>
#include <sys/syscall.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <vmxpi_ros/TitanInfo.h>
#include <vmxpi_ros/UniqueID.h>
#include <vmxpi_ros/Int.h>
#include <vmxpi_ros/StopMode.h>
#include <vmxpi_ros/LimitSwitch.h>
#include <vmxpi_ros/MotorSpeed.h>

class TitanDriverROSWrapper 
{
    private:
        std::unique_ptr<TitanDriver> Titan;
        VMXCAN *can;
        ros::ServiceServer stop_motor_server, test_server, enable_server, disable_server, reset_server, motor_stop_mode_server,
                            motor_speed;
        ros::Publisher uniqueid_pub, info_publisher, motor0freq_pub, motor1freq_pub, motor2freq_pub, motor3freq_pub, 
                        motor0speed_pub, motor1speed_pub, motor2speed_pub, motor3speed_pub,
                        enc0_pub, enc1_pub, enc2_pub, enc3_pub,
                        encdist0_pub, encdist1_pub, encdist2_pub, encdist3_pub,
                        rpm0_pub, rpm1_pub, rpm2_pub, rpm3_pub,
                        limitswitch_pub, mcutemp_pub;
        
        
        
        std::thread run1th, run2th;
    public:
        TitanDriverROSWrapper(ros::NodeHandle *nh, VMXPi *vmx);
        ~TitanDriverROSWrapper();
        
        // Sets specified motor's speed, valid values (0 - 1.0)
        bool SetMotorSpeed(vmxpi_ros::MotorSpeed::Request &req, vmxpi_ros::MotorSpeed::Response &res);
        
        // Stop all motors when service is called
        bool callbackStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        
        // Class function to stop specified motor
        void StopMotor(int motor);

        // Class function to stop all motors
        void StopAllMotors();
        
        // Enables Titan by sending an enabled flag through object's run loop
        bool Enable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // Disables Titan by sending a disabled flag through object's run loop
        bool Disable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    
        // Resets specified encoder's count
        bool ResetEncoder(vmxpi_ros::Int::Request &req, vmxpi_ros::Int::Response &res);
        
        // Resets all encoder counts
        bool ResetAllEncoders(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        /* Sets specified motor's stop mode
         * mode: (0 = coast, 1 = break) */
        bool SetMotorStopMode(vmxpi_ros::StopMode::Request &req, vmxpi_ros::StopMode::Response &res);

        // --- Publishers --- //
        
        /* Publish unique ID to /titan/unique_id
         * word1: hex value
         * word2: hex value
         * word3: hex value
         * Format: <word1>.<word2>.<word3>      */
        void PubUniqueID();

        /* Publish Titan's info (version, hardware) to /titan/info
         * deviceID: int32 (1-63)
         * verMaj: int32 (version format is <verMaj>.<verMin>.<verBuild>)
         * verMin: int32
         * verBuild: int32
         * hardware: int32 (1 = Titan Quad, 2 = Titan Small)
         * hardwareRev: int32                                       */
        void PubTitanInfo();

        // Publish motor speeds (0 - 1.0) to /titan/motorX/speed
        void PubMotorSpeed(uint8_t motor);
        
        // Publish motor frequencies to /titan/motorX/freq
        void PubMotorFreq();
        
        void GetCountRPM();
        
        // Publish encoder counts to /titan/encoderX/count
        void PubEncoderCount(int8_t encoder);
        
        // Publish encoder distances to /titan/encoderX/distance
        void PubEncoderDist(int8_t encoder);
        
        // Publish encoder RPM to /titan/motorX/rpm
        void PubRPM(uint8_t motor);

        /* Publish limit switch values to /titan/limit_switch
         * Sent as a bool array: {M0 L H, M0 L L, M1 L H, M1 L L, M2 L H, M2 L L, M3 L H, M3 L L} 
         * L = Low limit switch, H = High limit switch */
        void PubLimitSwitch();

        // Publish MCU (CPU) temp to /titan/mcu_temp
        void PubMCUTemp();
        
        void Run1_t();
        void Run2_t();
}; 


