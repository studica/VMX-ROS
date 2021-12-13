#include "IOwd_ros.h"

IOWatchdogROS::IOWatchdogROS(ros::NodeHandle *nh, VMXPi *vmx) : 
  io{&vmx->getIO()}, status{true} {
  VMXErrorCode vmxerr;
  bool enabled = true;
  if (!io->SetWatchdogTimeoutPeriodMS(250, &vmxerr)) // set the watchdog timeout period to 250ms
    DisplayVMXError(vmxerr);

	if (!io->SetWatchdogEnabled(enabled, &vmxerr)) // enable the watchdog
    DisplayVMXError(vmxerr);
    
  toggleio = nh->advertiseService("io_watchdog/status", &IOWatchdogROS::SetStatus, this);
  
  runth = std::thread(&IOWatchdogROS::Run_t, this);
}

IOWatchdogROS::~IOWatchdogROS() {
  runth.join();
}

bool IOWatchdogROS::SetStatus(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  status = req.data;
  res.success = true;
  if (status)
    res.message = "IO Watchdog enabled";
  else
    res.message = "IO Watchdog disabled";
  return true;
}

void IOWatchdogROS::Disable() {
  VMXErrorCode vmxerr;
  if(io->ExpireWatchdogNow(&vmxerr))
    DisplayVMXError(vmxerr);
}

void IOWatchdogROS::Run_t() {
  ros::Rate r(50);
  ROS_INFO_STREAM("IO watchdog thread: " << syscall(SYS_gettid));
  while (ros::ok()) { 
    VMXErrorCode vmxerr;
    bool expired;
    
    if (status) { // if enabled by the user, feed the watchdog
      if (!io->FeedWatchdog(&vmxerr)) {
        ROS_WARN("Error Feeding Watchdog.");
        DisplayVMXError(vmxerr);
      }
    }
    
    if (!io->GetWatchdogExpired(expired, &vmxerr)) // check if the watchdog has expired
      DisplayVMXError(vmxerr);
    if (expired)
      ROS_INFO_STREAM("IO Watchdog Expired");
    r.sleep();
  }
}
