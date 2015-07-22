#ifndef LIBDRONE_BEBOP_CONSTANTS_H
#define LIBDRONE_BEBOP_CONSTANTS_H

#include <string>

#include <drones/bebop/types.h>

namespace bebop
{
	const std::string DEFAULT_IP   = "192.168.42.1";
	const int DISCOVERY_PORT       = 44444;
	const int D2C_PORT			   = 43210;
	const int FULL_NAVDATA_PORT	   = 56789;

	namespace discovery
	{
		const std::string controller_type = "custom";
		const std::string controller_name = "libdrone";
	}
}

#endif
