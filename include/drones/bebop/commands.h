#ifndef LIBDRONE_BEBOP_COMMANDS_H
#define LIBDRONE_BEBOP_COMMANDS_H

#include <types.h>
#include <commands.h>

namespace bebop
{
	namespace commands
	{
		struct id
		{
			static const int FLIP 				 = 0xBEB0001;
			static const int CAMERA_ORIENTATION  = 0xBEB0002;
			static const int TAKE_PICTURE        = 0xBEB0003;
			static const int VIDEO			     = 0xBEB0004;
			static const int PICTURE_FORMAT		 = 0xBEB0005;
			static const int SET_HOME			 = 0xBEB0006;
			static const int NAVIGATE_HOME		 = 0xBEB0007;
		};

		struct flip : drone::command
		{
			enum direction
			{
				FRONT, LEFT, BACK, RIGHT
			};
			flip(direction d)
			: drone::command{id::FLIP, true, std::vector<boost::any>{d}} {}
		};

		struct camera_orientation : drone::command
		{
			camera_orientation(float tilt, float pan)
			: drone::command{id::CAMERA_ORIENTATION, false, std::vector<boost::any>{tilt, pan}} {}
		};

		struct take_picture : drone::command
		{
			take_picture()
			: drone::command{id::TAKE_PICTURE, false} {}
		};

		struct video : drone::command
		{
			enum action
			{
				STARTRECORDING,
				STOPRECORDING
			};
			video(action a)
			: drone::command{id::VIDEO, false, std::vector<boost::any>{a}} {}
		};

		struct set_home : drone::command
		{
			set_home(double lat, double lon)
			: drone::command{id::SET_HOME, false, std::vector<boost::any>{lat, lon}} {}
		};

		struct navigate_home : drone::command
		{
			navigate_home(bool start)
			: drone::command{id::NAVIGATE_HOME, true, std::vector<boost::any>{start}} {}
		};
	}
}

#endif
