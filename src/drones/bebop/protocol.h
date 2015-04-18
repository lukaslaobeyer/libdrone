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
	}

	namespace navdata_ids
	{
		const navdata_id wifi_rssi{0, 5, 7};

		const navdata_id altitude{1, 4, 8};
		const navdata_id attitude{1, 4, 6};
		const navdata_id speed{1, 4, 5};
		const navdata_id gps{1, 4, 4};

		const navdata_id camera_orientation{1, 25, 0};

		const navdata_id flattrim{1, 4, 0};
		const navdata_id flying_state{1, 4, 1};
		const navdata_id alert_state{1, 4, 2};
		const navdata_id homenavigation_state{1, 4, 3};

		const navdata_id picture_taken{1, 8, 0};
		const navdata_id video_recording_state{1, 8, 1};

		const navdata_id max_altitude{1, 6, 0};
		const navdata_id max_tilt{1, 6, 1};

		const navdata_id max_vertical_speed{1, 12, 0};
		const navdata_id max_rotation_speed{1, 12, 1};
		const navdata_id hull_protection{1, 12, 2};
		const navdata_id outdoor{1, 12, 3};

		const navdata_id picture_format{1, 20, 0};
		const navdata_id whitebalance_mode{1, 20, 1};
		const navdata_id picture_exposition{1, 20, 2};
		const navdata_id picture_saturation{1, 20, 3};
		const navdata_id video_autorecord{1, 20, 5};

		const navdata_id gps_fix{1, 24, 2};
	}

	namespace command_ids
	{
		const navdata_id flattrim{1, 0, 0};
		const navdata_id takeoff{1, 0, 1};
		const navdata_id pcmd{1, 0, 2};
		const navdata_id land{1, 0, 3};
		const navdata_id emergency{1, 0, 4};
		const navdata_id navigatehome{1, 0, 5};

		const navdata_id flip{1, 5, 0};

		const navdata_id enable_streaming{1, 21, 0};

		const navdata_id camera_orientation{1, 1, 0};

		const navdata_id take_picture{1, 7, 0};
		const navdata_id video{1, 7, 1};

		const navdata_id max_altitude{1, 2, 0};
		const navdata_id max_tilt{1, 2, 1};

		const navdata_id max_vertical_speed{1, 11, 0};
		const navdata_id max_rotation_speed{1, 11, 1};
		const navdata_id hull_protection{1, 11, 2};
		const navdata_id outdoor_flight{1, 11, 2};

		const navdata_id picture_format{1, 19, 0};
		const navdata_id whitebalance_mode{1, 19, 1};
		const navdata_id picture_exposition{1, 19, 2};
		const navdata_id picture_saturation{1, 19, 3};
		const navdata_id video_autorecord{1, 19, 5};

		const navdata_id set_home{1, 23, 0};
		const navdata_id reset_home{1, 23, 1};
	}
}

#endif
