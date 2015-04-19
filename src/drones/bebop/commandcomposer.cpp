#include "commandcomposer.h"

using namespace bebop::commands::create;

std::vector<boost::any> bebop::commands::create::pcmd(float roll, float pitch, float yaw, float gaz, drone::limits &limits)
{
	uint8_t activate_pitch_roll = 0;
	int8_t i8_roll = 0, i8_pitch = 0, i8_yaw = 0, i8_gaz = 0;

	if(roll == 0.0f && pitch == 0.0f)
	{
		activate_pitch_roll = 0;
	}
	else
	{
		// Convert pitch/roll from radians to value in range [-100:100] using the limits provided
		i8_roll = roll * (1/limits.angle) * 100.0f;
		i8_pitch = pitch * (1/limits.angle) * 100.0f;
	}

	// Convert yaw from radians/second to value between [-100:100]
	i8_yaw = yaw * (1/limits.yawspeed) * 100.0f;

	// Convert gaz from m/s to value betwen [-100:100]
	i8_gaz = gaz * (1/limits.vspeed) * 100.0f;

	std::vector<boost::any> args = {activate_pitch_roll, i8_roll, i8_pitch, i8_yaw, i8_gaz, 0.0f};

	return args;
}
