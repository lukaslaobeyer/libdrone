#ifndef ATTITUDECOMMAND_H
#define ATTITUDECOMMAND_H

#include "atcommand.h"

class AttitudeCommand : public ATCommand
{
	public:
		explicit AttitudeCommand(float phi = 0.0f, float theta = 0.0f, float gaz = 0.0f, float yaw = 0.0f) : ATCommand("PCMD", std::vector<std::string>{((theta == 0.0f && phi == 0.0f) ? "0" : "1"), ATCommand::_float(phi), ATCommand::_float(theta), ATCommand::_float(gaz), ATCommand::_float(yaw)}) {}
};

#endif
