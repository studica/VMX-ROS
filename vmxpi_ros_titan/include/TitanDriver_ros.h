#pragma once

#include "VMXPi.h"
#include <ros/ros.h>

#include "vmxpi_utils.h"
#include <stdint.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <sys/syscall.h>
#include <sys/types.h>

#include <vmxpi_ros/TitanInfo.h>
#include <vmxpi_ros/UniqueID.h>
#include <vmxpi_ros/LimitSwitch.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include "TitanConstants_ros.h"

class TitanDriver : public Utils {
    private:
        VMXCAN *can;
        VMXCANReceiveStreamHandle canrxhandles[1];
        double m_speed[4] = {0,0,0,0};
        int deviceID = 42;
        bool enabled;
        int frequency = 15600;
        int baseID;
        int autoRepeatPeriodInMilliseconds = 10;
        std::thread runth;
        
        double ENCODER_0_DISTANCE_PER_TICK = 0, ENCODER_1_DISTANCE_PER_TICK = 0,
                ENCODER_2_DISTANCE_PER_TICK = 0, ENCODER_3_DISTANCE_PER_TICK = 0;
        int32_t enc_count[4], rpm[4];
    public:

        TitanDriver(int deviceID, int frequency_set, VMXCAN *can_);

        TitanDriver(int deviceID, VMXCAN *can);
        
        ~TitanDriver();
        
        // Run loop on separate thread to disable/enable Titan
        void Run();
        
        // Configure specified motor's frequency
        void ConfigureMotor(uint8_t motor);
        
        // Set specified motor's speed (speed_set = 0 -> 1.0)
        void SetMotorSpeed(double speed_set, int motor);

        // Stop specified motor
        void StopMotor(uint8_t motor);

        // Stop all motors (todo: remove hardcoded motors)
        void StopAllMotors();
        
        // Reset encoder count for specified motor
        void ResetEncoder(uint8_t motor);
        
        // Resets all encoder counts
        void ResetAllEncoders();

        // Set the stop mode on the specified motor 
        // mode: (0 = coast, 1 = break)
        void SetMotorStopMode(uint8_t motor, int8_t mode);

        // Get Titan's Unique ID
        void GetUniqueID(vmxpi_ros::UniqueID *res_msg);
        
        // Get Titan's firmware and hardware information 
        void GetInfo(vmxpi_ros::TitanInfo *res_msg);
        
        // Get all motors' frequency
        void GetMotorFreq(std_msgs::UInt16 res_msg[]);
        
        void GetCountRPM(int8_t motor);

        /* Get the count of the specified encoder, note this
         * is not the encoder distance */
        void GetEncoderCount(std_msgs::Int32 *res_msg, int8_t encoder);
        
        /* Configure the encoder's distance per tick
         * For more info on this: https://docs.wsr.studica.com/en/latest/docs/Sensors/encoders.html 
         * encoderPPR = encoder pulse per ratio
         * gearRatio = rotation speed of motor to rotation speed of wheel */
        void ConfigureEncoder(int8_t encoder, double radius, double encoderPPR, double gearRatio);
        
        // Calculate the encoder's distance based on its current count
        void GetEncoderDist(std_msgs::Float32 *res_msg, int8_t encoder);
        
        // Get the RPM of the specified encoder
        void GetRPM(std_msgs::Int16 *res_msg, uint8_t motor);
        
        /* Get the limit switch values for each motor as a boolean value
         * {M0 L H, M0 L L, M1 L H, M1 L L, M2 L H, M2 L L, M3 L H, M3 L L}
         * L = low limit switch, H = high limit switch */
        void GetLimitSwitch(vmxpi_ros::LimitSwitch *res_msg);
        
        // Get the current MCU (CPU) temperature
        void GetMCUTemp(std_msgs::Float32 *res_msg);
        
        // Get specified motor's speed
        void GetSpeed(std_msgs::Float32 *res_msg, uint8_t motor);
        
        // Sets the current flag being sent by the run loop to enabled
        void Enable();
        
        // Sets the current flag being sent by the run loop to disabled
        void Disable();
    
    private:
        bool CheckCANStatus(VMXErrorCode vmxerr) {
            VMXCANBusStatus can_bus_status;
            if (!can->GetCANBUSStatus(can_bus_status, &vmxerr)) {
                ROS_INFO("Error getting CAN BUS Status.");
                return false;
                DisplayVMXError(vmxerr);
            } else {
                if(can_bus_status.busWarning) 
                    ROS_INFO("CAN Bus Warning.\n");
                if(can_bus_status.busPassiveError) 
                    ROS_INFO("CAN Bus in Passive mode due to errors.");
                if(can_bus_status.busOffError) 
                    ROS_INFO("CAN Bus Transmitter Off due to errors.");
                if(can_bus_status.transmitErrorCount > 0) 
                    ROS_INFO("CAN Bus Tx Error Count:  %d", can_bus_status.transmitErrorCount);
                if(can_bus_status.receiveErrorCount > 0)
                    ROS_INFO("CAN Bus Rx Error Count:  %d", can_bus_status.receiveErrorCount);
                if(can_bus_status.busOffCount > 0)
                    ROS_INFO("CAN Bus Tx Off Count:  %d", can_bus_status.busOffCount);
                if(can_bus_status.txFullCount > 0)
                    ROS_INFO("CAN Bus Tx Full Count:  %d", can_bus_status.txFullCount);
                if(can_bus_status.hwRxOverflow) 
                    ROS_INFO("CAN HW Receive Overflow detected.");
                if(can_bus_status.swRxOverflow) 
                    ROS_INFO("CAN SW Receive Overflow detected.");
                if(can_bus_status.busError) 
                    ROS_INFO("CAN Bus Error detected.");
                if(can_bus_status.wake) 
                    ROS_INFO("CAN Bus Wake occured.");
                if(can_bus_status.messageError) 
                    ROS_INFO("CAN Message Error detected.");
                else {
                    //ROS_INFO("CAN working");
                    return true;
                }
                return false;         
            }
        }
        
        // Blackboard read function utilizing VMXPi's blackboard functionality
        void ReadCANBlackboard(VMXCANReceiveStreamHandle handle, int msgID, VMXCANTimestampedMessage* data, uint32_t length, uint64_t timestamp, VMXErrorCode vmxerr);
};

