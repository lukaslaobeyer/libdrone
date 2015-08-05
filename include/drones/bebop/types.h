#ifndef LIBDRONE_BEBOP_TYPES_H
#define LIBDRONE_BEBOP_TYPES_H

#include <types.h>

#define BEBOP_NAVDATA_BUFFER_SIZE 65000

namespace bebop
{
	typedef std::array<char, BEBOP_NAVDATA_BUFFER_SIZE> d2cbuffer;

	struct navdata_extension
	{
		float ultrasound_height; // Height as measured by the ultrasonic sensor in m
		float pressure; // Pressure in Pascal
		Eigen::Vector2f horiz_speed; // Horizontal speed in m/s; x - y
		Eigen::Vector3f ned_speed; // NED speed; x - y - z
		float vbat; // Battery voltage in V
		float gps_speed; // Sped in m/s as measured by GPS
		float gps_bearing; // GPS bearing in degrees
		float gps_accuracy; // GPS accuracy in degrees
		float gps_eph;
		float gps_epv;
	};

	struct navdata : drone::navdata
	{
		Eigen::Vector2f cameraorientation; // tilt, pan, unknown units TODO: units!
		bool full;
		bool gps_fix;
		double latitude; // latitude from GPS in decimal degrees
		double longitude; // longitude from GPS in decimal degrees
		double gps_altitude; // altitude from GPS in meters
		int gps_sats; // Number of available GPS sattelites
		navdata_extension full_navdata; // Full navdata!
	};

	enum pictureformat
	{
		PICFMT_RAW = 0,
		PICFMT_JPEG,
		PICFMT_SNAPSHOT,
		PICFMT_JPEG_FISHEYE
	};

	enum whitebalancemode
	{
		WB_AUTO = 0,
		WB_TUNGSTEN,
		WB_DAYLIGHT,
		WB_CLOUDY,
		WB_COOL_WHITE
	};

	enum antiflickermode
	{
		AF_AUTO = 0,
		AF_50HZ,
		AF_60HZ
	};
}

#endif
