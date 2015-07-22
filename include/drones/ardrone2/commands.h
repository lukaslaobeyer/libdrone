#ifndef LIBDRONE_ARDRONE2_COMMANDS_H
#define LIBDRONE_ARDRONE2_COMMANDS_H

#include <types.h>
#include <commands.h>

namespace ardrone2
{
	namespace commands
	{
		struct id
		{
			static const int MAGNETOCALIB = 0xAD20001;
			static const int CONFIG	      = 0xAD20002;
			static const int RECORDONUSB  = 0xAD20003;
			static const int SWITCHVIEW   = 0xAD20004;
			static const int FLIP		  = 0xAD20005;
		};

		struct magnetometercalibration : drone::command
		{
			magnetometercalibration()
			: drone::command{id::MAGNETOCALIB, true} {}
		};

		struct configuration : drone::command
		{
			configuration(std::string key, std::string value)
			: drone::command{id::CONFIG, false, std::vector<boost::any>{key, value}} {}
		};

		struct recordonusb : drone::command
		{
			recordonusb(bool record)
			: drone::command{id::RECORDONUSB, false, std::vector<boost::any>{record}} {}
		};

		struct switchview : drone::command
		{
			enum view
			{
				FRONT,
				BOTTOM
			};

			switchview(view v)
			: drone::command{id::SWITCHVIEW, false, std::vector<boost::any>{v}} {}
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
	}
}

#endif
