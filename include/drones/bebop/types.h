#ifndef LIBDRONE_BEBOP_TYPES_H
#define LIBDRONE_BEBOP_TYPES_H

#include <types.h>

#define BEBOP_NAVDATA_BUFFER_SIZE 65000

namespace bebop
{
	typedef std::array<char, BEBOP_NAVDATA_BUFFER_SIZE> d2cbuffer;

	struct navdata : drone::navdata
	{
		Eigen::Vector2f cameraorientation; // tilt, pan, unknown units TODO: units!
		bool gps_fix;
		double latitude; // latitude from GPS in decimal degrees
		double longitude; // longitude from GPS in decimal degrees
		double gps_altitude; // altitude from GPS in meters
		int gps_sats; // Number of available GPS sattelites
	};
}

#endif
