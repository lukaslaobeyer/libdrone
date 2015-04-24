#ifndef LIBDRONE_BEBOP_COMMANDCOMPOSER_H
#define LIBDRONE_BEBOP_COMMANDCOMPOSER_H

#include <types.h>
#include "protocol.h"
#include <drones/bebop/commands.h>

#include <vector>
#include <boost/any.hpp>

namespace bebop
{
	namespace commands
	{
		namespace create // Functions in this namespace create the list of arguments for the Bebop commands
		{
			std::vector<boost::any> pcmd(float roll, float pitch, float yaw, float gaz, drone::limits &limits);
			std::vector<boost::any> pcmdrel(float roll, float pitch, float yaw, float gaz, drone::limits &limits);
			std::vector<boost::any> camera_orientation(float tilt, float pan);
			std::vector<boost::any> flip(bebop::commands::flip::direction direction);
		}
	}
}

#endif
