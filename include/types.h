#ifndef LIBDRONE_TYPES_H
#define LIBDRONE_TYPES_H

#include <Eigen/Dense>

#include <vector>
#include <iostream>
#include <boost/optional.hpp>

namespace drone
{
	struct navdata
	{
		bool flying;
		float batteryStatus; // battery charge level from 0 (no battery charge left) to 1 (battery full)
		float linkQuality; // communication link quality from 0 (no link) to 1 (perfect link)

		float altitude; // in meters
		Eigen::Vector3f attitude; // 0 (x-Axis): pitch/theta, 1 (y-Axis): roll/psi, 2 (z-Axis): yaw/phi; in radians
		Eigen::Vector3f linearvelocity; // vx, vy, vz in m/s (warning: z may always be 0 with AR.Drone 2.0)
	};
	
	struct limits
	{
	    float angle; // Maximum pitch and roll angle in radians
	    float yawspeed; // Maximum yaw (rotation) speed in radians/second
	    float vspeed; // Maximum vertical speed in m/s
	};

	enum connectionstatus
	{
		CONNECTION_ESTABLISHED,
		CONNECTION_FAILED,
		ALREADY_CONNECTED,
		EXCEPTION_OCCURRED
	};
}

#endif
