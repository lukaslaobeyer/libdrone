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
		bool needsArming;
		std::vector<boost::any> parameters;
	};

	namespace commands
	{
		struct id
		{
			static const int EMERGENCY   = 0x00;
			static const int TAKEOFF  	 = 0x01;
			static const int LAND     	 = 0x02;
			static const int ATTITUDE 	 = 0x03;
			static const int ATTITUDEREL = 0x04;
			static const int FTTRIM 	 = 0x05;
		};

		struct emergency : drone::command
		{
			emergency()
			: drone::command{id::EMERGENCY, false} {}
		};

		struct takeoff : drone::command
		{
			takeoff()
			: drone::command{id::TAKEOFF, true} {}
		};

		struct land : drone::command
		{
			land()
			: drone::command{id::LAND, false} {}
		};

		struct fttrim : drone::command
		{
			fttrim()
			: drone::command{id::FTTRIM, false} {}
		};

		struct attitude : drone::command
		{
			explicit attitude(Eigen::Vector3f attitude = Eigen::Vector3f(0, 0, 0), float vspeed = 0)
			: drone::command{id::ATTITUDE, false, std::vector<boost::any>{boost::any(attitude), boost::any(vspeed)}} {}
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

		struct attituderel : drone::command
		{
			explicit attituderel(Eigen::Vector3f attitude = Eigen::Vector3f(0, 0, 0), float vspeed = 0)
			: drone::command{id::ATTITUDEREL, false, std::vector<boost::any>{boost::any(attitude), boost::any(vspeed)}} {}
		};
	}
}

#endif
