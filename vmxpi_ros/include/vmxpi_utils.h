#pragma once

#include "VMXPi.h"

class Utils {
	public:
		void DisplayVMXError(VMXErrorCode vmxerr) {
			const char *p_err_description = GetVMXErrorString(vmxerr);
			ROS_INFO("VMXError %d:  %s\n", vmxerr, p_err_description);
		}
};
