#pragma once

#include <ros/ros.h>
#include "VMXPi.h"
#include "vmxpi_utils.h"
#include <unistd.h>
#include <sys/syscall.h>
#include <std_srvs/SetBool.h>

class IOWatchdogROS : public Utils {
  private:
    VMXIO *io; // IO class of the VMXPi
    bool status;
    std::thread runth;
    
    // service to enable the IO Watchdog loop
    ros::ServiceServer toggleio;
    
  public:
    IOWatchdogROS(ros::NodeHandle *nh, VMXPi *vmx);
    ~IOWatchdogROS();
    
    bool SetStatus(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    
    void Run_t();
    void Disable();
};
