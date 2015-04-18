#ifndef LIBDRONE_BEBOP_PROTOCOL_H
#define LIBDRONE_BEBOP_PROTOCOL_H

#include <string>
#include <vector>

namespace bebop
{
	struct navdata_id
	{
		uint8_t dataDevice;
		uint8_t dataClass;
		uint16_t dataID;
		std::string dataTypes; // Datatype string for navdata_id (similar to the format characters in python's struct library):
							   // b: int8_t
							   // B: uint8_t
							   // h: int16_t
							   // H: uint16_t
							   // f: float
							   // d: double

		bool operator==(const navdata_id& rhs) const
		{
			if((dataDevice == rhs.dataDevice) &&
			   (dataClass == rhs.dataClass) &&
			   (dataID == rhs.dataID))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	};

	struct frameheader
	{
		uint8_t type;   // Packet type: 2 for regular command
		uint8_t id;     // Packet ID: 10 for regular frame, 11 for answering ack requests
		uint8_t seq;    // Sequence number of the packet
		uint32_t size;  // Size of the packet
	};

	namespace frametype
	{
		const uint8_t UNINITIALIZED = 0;
		const uint8_t ACK = 1;
		const uint8_t DATA = 2;
		const uint8_t LOW_LATENCY = 3;
		const uint8_t DATA_WITH_ACK = 4;
	}

	namespace frameid
	{
		const uint8_t PING = 0;
		const uint8_t PONG = 1;
		const uint8_t NAVDATA = 0x7f;
		const uint8_t COMMAND = 10;
	}

	namespace navdata_ids
	{
		const navdata_id wifi_rssi{0, 5, 7, "h"}; // in dBm

		const navdata_id altitude{1, 4, 8, "d"}; // in meters
		const navdata_id attitude{1, 4, 6, "fff"}; // roll, pitch, yaw
		const navdata_id speed{1, 4, 5, "fff"}; // x, y, z in m/s
		const navdata_id gps{1, 4, 4, "ddd"}; // latitude, longitude in decimal degrees and altitude in m

		const navdata_id camera_orientation{1, 25, 0, "bb"}; // tilt and pan in range [-100:100]

		const navdata_id flattrim{1, 4, 0, ""};
		const navdata_id flying_state{1, 4, 1, "B"}; //TODO: decode states
		const navdata_id alert_state{1, 4, 2, "B"};
		const navdata_id homenavigation_state{1, 4, 3, "B"};

		const navdata_id picture_taken{1, 8, 0, "B"}; // 1 if picture taken
		const navdata_id video_recording_state{1, 8, 1, "B"}; //TODO: decode state

		const navdata_id max_altitude{1, 6, 0, "fff"}; // current, min, max in m
		const navdata_id max_tilt{1, 6, 1, "fff"}; // current, min, max in deg

		const navdata_id max_vertical_speed{1, 12, 0, "fff"}; // current, min, max in m/s
		const navdata_id max_rotation_speed{1, 12, 1, "fff"}; // current, min, max in deg/s
		const navdata_id hull_protection{1, 12, 2, "B"}; // 1 if hull present
		const navdata_id outdoor{1, 12, 3, "B"}; // 1 if flying outdoors

		const navdata_id picture_format{1, 20, 0, "B"};
		const navdata_id whitebalance_mode{1, 20, 1, "B"};
		const navdata_id picture_exposition{1, 20, 2, "fff"}; // current, min, max
		const navdata_id picture_saturation{1, 20, 3, "fff"}; // current, min, max
		const navdata_id video_autorecord{1, 20, 5, "BB"}; // 1: 1 if enabled, 2: mass storage ID

		const navdata_id gps_fix{1, 24, 2, "B"}; // 1 if fix acquired
	}

	namespace command_ids
	{
		const navdata_id flattrim{1, 0, 0, ""};
		const navdata_id takeoff{1, 0, 1, ""};
		const navdata_id pcmd{1, 0, 2, "Bbbbbf"}; // 1: 1 to activate roll/pitch movement; 2-5: roll, pitch, yaw, gaz in range [-100:100]; 6: unused
		const navdata_id land{1, 0, 3, ""};
		const navdata_id emergency{1, 0, 4, ""};
		const navdata_id navigatehome{1, 0, 5, "B"}; // 1 to navigate home, 0 to stop navigating autonomously

		const navdata_id flip{1, 5, 0, "B"}; //TODO: direction of flip (probably 0: front; 1: back; 2: right; 3: left)

		const navdata_id enable_streaming{1, 21, 0, "B"}; // 1 to enable video streaming, 0 to disable

		const navdata_id camera_orientation{1, 1, 0, "bb"}; // tilt, pan in deg

		const navdata_id take_picture{1, 7, 0, "B"}; // Mass storage ID (whatever that means, Parrot! ...) TODO: find this out
		const navdata_id video{1, 7, 1, "BB"}; // 1: 1 to start recording, 0 to stop; 2: Mass storage ID

		const navdata_id max_altitude{1, 2, 0, "f"}; // Maximum altitude in m
		const navdata_id max_tilt{1, 2, 1, "f"}; // Maximum tilt in deg

		const navdata_id max_vertical_speed{1, 11, 0, "f"}; // Maximum vertical speed in m/s
		const navdata_id max_rotation_speed{1, 11, 1, "f"}; // Maximum rotation speed in deg/s
		const navdata_id hull_protection{1, 11, 2, "B"}; // 1 for hull protection
		const navdata_id outdoor_flight{1, 11, 2, "B"}; // 1 for outdoor flight

		const navdata_id picture_format{1, 19, 0, "B"}; // 0: raw; 1: 4:3 JPEG; 2: 16:9 snapshot TODO: verify
		const navdata_id whitebalance_mode{1, 19, 1, "B"}; // 0: auto; 1: tungsten; 2: daylight; 3: cloudy; 4: cool white TODO: verify
		const navdata_id picture_exposition{1, 19, 2, "f"}; // Exposition, by default in range [-3:3]
		const navdata_id picture_saturation{1, 19, 3, "f"}; // Saturation, by default in range [-100:100]
		const navdata_id video_autorecord{1, 19, 5, "BB"}; // 1: 1 for video autorecord enable, 0 otherwise; 2: Mass storage ID

		const navdata_id set_home{1, 23, 0, "ddd"}; // Latitude, longitude in decimal degrees; altitude in meters
		const navdata_id reset_home{1, 23, 1, ""};
	}
}

#endif
