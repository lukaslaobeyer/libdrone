#ifndef LIBDRONE_COMMANDS_H
#define LIBDRONE_COMMANDS_H

#include <types.h>

#include <Eigen/Dense>
#include <cstdint>
#include <boost/spirit/home/support/detail/hold_any.hpp>

namespace drone
{
	struct command
	{
		int command;
		std::vector<boost::spirit::hold_any> parameters;
	};

	namespace commands
	{
		struct id
		{
			const int EMERGENCY = 0x00;
			const int TAKEOFF  	= 0x01;
			const int LAND     	= 0x02;
			const int ATTITUDE 	= 0x03;
			const int FTTRIM	= 0x04;
		};

		struct emergency : command
		{
			emergency()
			: command{id::EMERGENCY} {}
		};

		struct takeoff : command
		{
			takeoff()
			: command{id::TAKEOFF} {}
		};

		struct land : command
		{
			land()
			: command{id::LAND} {}
		};

		struct fttrim : command
		{
			fttrim()
			: command{id::FTTRIM} {}
		};

		struct attitude : command
		{
			explicit attitude(Eigen::Vector3f attitude = Eigen::Vector3f(0, 0, 0), float vspeed = 0)
			: command{id::ATTITUDE, std::vector<boost::spirit::hold_any>{attitude, vspeed}} {}
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
