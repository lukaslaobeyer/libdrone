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
							   // i: int32_t
							   // I: uint32_t
							   // l: int64_t
							   // L: uint64_t
							   // f: float
							   // d: double
							   // s: string (UTF-8, null terminated)

		bool acked; // Command expects an 'acknowledge' signal from Bebop after being sent.
		                   // This has no meaning for navdata and is only used for commands.

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
		const uint8_t NAVDATA_WITH_ACK = 0x7e;
		const uint8_t ACK_RESPONSE = 0xFE;
		const uint8_t VIDEO_ACK_RESPONSE = 13;
		const uint8_t COMMAND = 10;
		const uint8_t COMMAND_WITH_ACK = 11;
		const uint8_t COMMAND_ACK_RESPONSE = 139;
		const uint8_t VIDEO_WITH_ACK = 0x7D;
	}

	namespace navdata_ids
	{
		const navdata_id disconnection{0, 1, 0, "B", false}; // Drone says goodbye. If 0, cause is off button press. Otherwise, cause is unknown.

		const navdata_id all_states_sent{0, 5, 0, "", false};

		const navdata_id wifi_rssi{0, 5, 7, "h", false}; // in dBm
		const navdata_id battery_status{0, 5, 1, "B", false}; // Battery charge percentage

		const navdata_id date{0, 5, 4, "s", false}; // Date in ISO-8601 format
		const navdata_id time{0, 5, 5, "s", false}; // Time in ISO-8601 format

		const navdata_id sensor_state{0, 5, 8, "BB", false};

		const navdata_id outdoor_wifi_state{0, 10, 0, "B", false}; // 1 if outdoor, 0 if indoor

		const navdata_id flightplan_state_changed{0, 12, 0, "IsI", false};
		const navdata_id flightplan_error{0, 12, 1, "B", false};

		const navdata_id magneto_calib_state{0, 14, 0, "BBBB", false};
		const navdata_id magneto_calib_required{0, 14, 1, "B", false};

		const navdata_id camera_fov{0, 15, 0, "f", false};

		const navdata_id massstorage_list{0, 5, 2, "Bs", false};
		const navdata_id massstorage_info{0, 5, 3, "BIIBBB", false};

		const navdata_id motor_state{1, 16, 2, "BB", false};
		const navdata_id motor_version{1, 16, 3, "s", false};
		const navdata_id motor_flightstatus{1, 16, 4, "HHI", false};
		const navdata_id motor_previouserror{1, 16, 5, "B", false};

		const navdata_id altitude{1, 4, 8, "d", false}; // in meters
		const navdata_id attitude{1, 4, 6, "fff", false}; // roll, pitch, yaw
		const navdata_id speed{1, 4, 5, "fff", false}; // x, y, z in m/s
		const navdata_id gps{1, 4, 4, "ddd", false}; // latitude, longitude in decimal degrees and altitude in m
		const navdata_id gps_sats{129, 3, 0, "B", false}; // number of available GPS satellites

		const navdata_id camera_orientation{1, 25, 0, "bb", false}; // tilt and pan in range [-100:100]

		const navdata_id flattrim{1, 4, 0, "", false};
		const navdata_id flying_state{1, 4, 1, "B", false}; // 0: landed; 1: taking off; 2: hovering; 3: flying; 4: landing; 5: emergency
		const navdata_id alert_state{1, 4, 2, "B", false}; // 0: no alert; 1: user emergency; 2: cut out alert (??); 3: critical battery level; 4: low battery; 5: angle too high
		const navdata_id homenavigation_state{1, 4, 3, "B", false};

		const navdata_id picture_state{1, 8, 2, "BB", false}; // 0 if ready, 1 if busy, 2 if "not available"; 0 if OK, 1 if unknown error, 2 if camera problem, 3 if memory full, 4 if low battery
		const navdata_id video_recording_state{1, 8, 3, "BB", false}; // same as picture_taken

		const navdata_id streaming_state{1, 22, 0, "B", false}; // 0: enabled; 1: disabled; 2: error

		const navdata_id max_altitude{1, 6, 0, "fff", false}; // current, min, max in m
		const navdata_id max_tilt{1, 6, 1, "fff", false}; // current, min, max in deg

		const navdata_id max_vertical_speed{1, 12, 0, "fff", false}; // current, min, max in m/s
		const navdata_id max_rotation_speed{1, 12, 1, "fff", false}; // current, min, max in deg/s
		const navdata_id hull_protection{1, 12, 2, "B", false}; // 1 if hull present
		const navdata_id outdoor{1, 12, 3, "B", false}; // 1 if flying outdoors

		const navdata_id picture_format{1, 20, 0, "B", false};
		const navdata_id whitebalance_mode{1, 20, 1, "B", false};
		const navdata_id picture_exposure{1, 20, 2, "fff", false}; // current, min, max
		const navdata_id picture_saturation{1, 20, 3, "fff", false}; // current, min, max
		const navdata_id video_autorecord{1, 20, 5, "BB", false}; // 1: 1 if enabled, 2: mass storage ID

		const navdata_id home_changed{1, 24, 0, "ddd", false};
		const navdata_id gps_fix{1, 24, 2, "B", false}; // 1 if fix acquired
	}

	namespace command_ids
	{
		const navdata_id getsettings{0, 2, 0, "", true}; // Tell the drone to send back all settings
		const navdata_id getstatus{0, 4, 0, "", true}; // Tell the drone to send back all status information (battery charge, flying state, etc.)

		const navdata_id reset{0, 2, 1, "", true}; // Reset all settings

		const navdata_id setdate{0, 4, 1, "s", true}; // Date in ISO-8601 format
		const navdata_id settime{0, 4, 2, "s", true}; // Time in ISO-8601 format

		const navdata_id piloting_mode{0, 8, 0, "B", true}; // 1 to enable piloting mode, 0 to disable

		const navdata_id outdoor_wifi_mode{0, 9, 0, "B", true}; // 1 for outdoor, 0 for indoor

		const navdata_id flightplan_start{0, 11, 0, "s", true}; // Start executing the flight plan. Argument: mavlink filename
		const navdata_id flightplan_pause{0, 11, 1, "", true};
		const navdata_id flightplan_stop{0, 11, 2, "", true};

		const navdata_id flattrim{1, 0, 0, "", true};
		const navdata_id takeoff{1, 0, 1, "", true};
		const navdata_id pcmd{1, 0, 2, "Bbbbbf", false}; // 1: 1 to activate roll/pitch movement; 2-5: roll, pitch, yaw, gaz in range [-100:100]; 6: unused
		const navdata_id land{1, 0, 3, "", true};
		const navdata_id emergency{1, 0, 4, "", true};
		const navdata_id navigatehome{1, 0, 5, "B", true}; // 1 to navigate home, 0 to stop navigating autonomously

		const navdata_id flip{1, 5, 0, "B", true}; //TODO: direction of flip (probably 0: front; 1: back; 2: right; 3: left)

		const navdata_id enable_streaming{1, 21, 0, "B", true}; // 1 to enable video streaming, 0 to disable

		const navdata_id camera_orientation{1, 1, 0, "bb", false}; // tilt, pan in deg

		const navdata_id take_picture{1, 7, 2, "", true};
		const navdata_id video{1, 7, 3, "BBBB", true}; // 1: 1 to start recording, 0 to stop; rest 0

		const navdata_id max_altitude{1, 2, 0, "f", true}; // Maximum altitude in m
		const navdata_id max_tilt{1, 2, 1, "f", true}; // Maximum tilt in deg

		const navdata_id max_vertical_speed{1, 11, 0, "f", true}; // Maximum vertical speed in m/s
		const navdata_id max_rotation_speed{1, 11, 1, "f", true}; // Maximum rotation speed in deg/s
		const navdata_id hull_protection{1, 11, 2, "B", true}; // 1 for hull protection
		const navdata_id outdoor_flight{1, 11, 3, "B", true}; // 1 for outdoor flight - WARNING: ignored by drone (??)

		const navdata_id picture_format{1, 19, 0, "I", true}; // 0: raw; 1: 4:3 JPEG; 2: 16:9 snapshot
		const navdata_id whitebalance_mode{1, 19, 1, "I", true}; // 0: auto; 1: tungsten; 2: daylight; 3: cloudy; 4: cool white
		const navdata_id picture_exposure{1, 19, 2, "f", true}; // Exposure, by default in range [-3:3]
		const navdata_id picture_saturation{1, 19, 3, "f", true}; // Saturation, by default in range [-100:100]
		const navdata_id video_autorecord{1, 19, 5, "BB", true}; // 1: 1 for video autorecord enable, 0 otherwise; 2: Mass storage ID

		const navdata_id set_home{1, 23, 0, "ddd", true}; // Latitude, longitude in decimal degrees; altitude in meters
		const navdata_id reset_home{1, 23, 1, "", true};

		const navdata_id antiflickering_mode{1, 29, 1, "I", true};

		const navdata_id stream_720p{1, 32, 0, "L", true};
	}
}

#endif
