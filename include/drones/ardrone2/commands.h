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
			const int MAGNETOCALIB = 0xAD20001;
			const int CONFIG	   = 0xAD20002;
			const int RECORDONUSB  = 0xAD20003;
			const int SWITCHVIEW   = 0xAD20004;
		};

		struct magnetometercalibration : drone::command
		{
			magnetometercalibration()
			: drone::command{id::MAGNETOCALIB} {}
		};

		struct configuration : drone::command
		{
			configuration(std::string key, std::string value)
			: drone::command{id::CONFIG, std::vector<boost::spirit::hold_any>{key, value}} {}
		};

		struct recordonusb : drone::command
		{
			recordonusb(bool record)
			: drone::command{id::RECORDONUSB, std::vector<boost::spirit::hold_any>{record}} {}
		};

		struct switchview : drone::command
		{
			enum view
			{
				FRONT,
				BOTTOM
			};

			switchview(view v)
			: drone::command{id::SWITCHVIEW, std::vector<boost::spirit::hold_any>{v}} {}
		};
	}
}

#endif
