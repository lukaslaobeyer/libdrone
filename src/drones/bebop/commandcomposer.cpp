#include "commandcomposer.h"

#include <boost/log/trivial.hpp>

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
		activate_pitch_roll = 1;

		// Convert pitch/roll from radians to value in range [-100:100] using the limits provided
		i8_roll = roll * (1/limits.angle) * 100.0f;
		i8_pitch = pitch * (1/limits.angle) * 100.0f;
	}

	// Convert yaw from radians/second to value between [-100:100]
	i8_yaw = yaw * (1/limits.yawspeed) * 100.0f;

	// Convert gaz from m/s to value between [-100:100]
	i8_gaz = gaz * (1/limits.vspeed) * 100.0f;

	std::vector<boost::any> args = {activate_pitch_roll, i8_roll, i8_pitch, i8_yaw, i8_gaz, 0.0f};

	return args;
}

std::vector<boost::any> bebop::commands::create::pcmdrel(float roll, float pitch, float yaw, float gaz, drone::limits &limits)
{
	uint8_t activate_pitch_roll = 0;
	int8_t i8_roll = 0, i8_pitch = 0, i8_yaw = 0, i8_gaz = 0;

	if(roll == 0.0f && pitch == 0.0f)
	{
		activate_pitch_roll = 0;
	}
	else
	{
		activate_pitch_roll = 1;

		// Convert pitch/roll from -1.0 to 1.0 to value in range [-100:100] using the limits provided
		i8_roll = roll * 100.0f;
		i8_pitch = pitch * 100.0f;
	}

	// Convert yaw from from -1.0 to 1.0 to value between [-100:100]
	i8_yaw = yaw * 100.0f;

	// Convert gaz from from -1.0 to 1.0 to value between [-100:100]
	i8_gaz = gaz * 100.0f;

	std::vector<boost::any> args = {activate_pitch_roll, i8_roll, i8_pitch, i8_yaw, i8_gaz, 0.0f};

	return args;
}

std::vector<boost::any> bebop::commands::create::camera_orientation(float tilt, float pan)
{
	int8_t i8_tilt = (int8_t) tilt;
	int8_t i8_pan = (int8_t) pan;

	std::vector<boost::any> args = {i8_tilt, i8_pan};

	return args;
}

std::vector<boost::any> bebop::commands::create::flip(bebop::commands::flip::direction direction)
{
	uint8_t u8_direction = 0;

	switch(direction)
	{
	case bebop::commands::flip::direction::FRONT:
		u8_direction = 0;
		break;
	case bebop::commands::flip::direction::BACK:
		u8_direction = 1;
		break;
	case bebop::commands::flip::direction::RIGHT:
		u8_direction = 2;
		break;
	case bebop::commands::flip::direction::LEFT:
		u8_direction = 3;
		break;
	}

	std::vector<boost::any> args = {u8_direction};

	return args;
}
