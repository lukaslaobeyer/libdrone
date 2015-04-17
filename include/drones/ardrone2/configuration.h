#ifndef ARDRONE2_CONFIGURATION_H
#define ARDRONE2_CONFIGURATION_H

namespace ardrone2
{
    struct configuration // AR.Drone 2.0 configuration initialized to default values
    {
    	float pitch_roll_max = 0.25f;     // Maximum pitch/roll angle in radians. Should be between 0 and 0.5.
    	float altitude_max = 5;           // Maximum drone altitude in m.
    	float vertical_speed_max = 1;     // Maximum vertical speed in m/s. Recommended value: 0.2 - 2.
    	float yaw_speed_max = 4.5;        // Maximum yaw speed in radians/second. Recommended value: 0.7 rad/s - 6.1 rad/s.
    	bool outdoor_flight = false;      // Should be set to true if flying outdoors. (Enables the AR.Drone's wind calculator.)
    	bool no_hull = false;             // Should be set to true if flying without hull / with the outdoor hull.
    };
}

#endif
