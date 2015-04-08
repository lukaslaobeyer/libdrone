#ifndef LIBDRONE_COMMANDS_H
#define LIBDRONE_COMMANDS_H

#include <types.h>

#include <Eigen/Dense>
#include <cstdint>
#include <boost/any.hpp>

namespace drone
{
	struct command
	{
		int command;
		std::vector<boost::any> parameters;
	};

	namespace commands
	{
		struct id
		{
			static const int EMERGENCY  = 0x00;
			static const int TAKEOFF  	= 0x01;
			static const int LAND     	= 0x02;
			static const int ATTITUDE 	= 0x03;
			static const int FTTRIM 	= 0x04;
		};

		struct emergency : drone::command
		{
			emergency()
			: drone::command{id::EMERGENCY} {}
		};

		struct takeoff : drone::command
		{
			takeoff()
			: drone::command{id::TAKEOFF} {}
		};

		struct land : drone::command
		{
			land()
			: drone::command{id::LAND} {}
		};

		struct fttrim : drone::command
		{
			fttrim()
			: drone::command{id::FTTRIM} {}
		};

		struct attitude : drone::command
		{
			explicit attitude(Eigen::Vector3f attitude = Eigen::Vector3f(0, 0, 0), float vspeed = 0)
			: drone::command{id::ATTITUDE, std::vector<boost::any>{boost::any(attitude), boost::any(vspeed)}} {}
			/*
			 * attitude (Eigen::Vector3f):
			 *   0 (x-Axis):
			 *   pitch/theta (negative for forward pitch, positive for backwards pitch) (in radians)
			 *   1 (y-Axis):
			 *   roll/psi (negative for left roll, positive for right roll) (in radians)
			 *   2 (z-Axis):
			 *   yaw/phi (positive for clockwise, negative for counterclockwise) (in radians/second)
			 *
			 * vspeed (vertical speed) (float):
			 * vertical speed in m/s (negative for descending, positive for ascending)
			 */
		};
	}
}

#endif
