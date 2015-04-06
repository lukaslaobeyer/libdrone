#ifndef LIBDRONE_ARDRONE2_TYPES_H
#define LIBDRONE_ARDRONE2_TYPES_H

#include <types.h>

namespace ardrone2
{
	struct navdata : drone::navdata
	{
		float pressure; // Pressure reading
		Eigen::Vector3f acceleration; // x, y, z acceleration in m/s^2
		Eigen::Vector3f magnetometer; // x, y, z data from the magnetometer
	};
}

#endif
