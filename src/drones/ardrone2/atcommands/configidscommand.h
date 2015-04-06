#ifndef CONFIGIDSCOMMAND_H
#define CONFIGIDSCOMMAND_H

#include "atcommand.h"
#include <types.h>

class ConfigIDSCommand : public ATCommand
{
	public:
		explicit ConfigIDSCommand() : ATCommand("CONFIG_IDS", std::vector<std::string>{ATCommand::_string(ardrone2::app_id::SESSION_ID), ATCommand::_string(ardrone2::app_id::PROFILE_ID), ATCommand::_string(ardrone2::app_id::APPLICATION_ID)}) {}
};

#endif
