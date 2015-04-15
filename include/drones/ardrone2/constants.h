#ifndef LIBDRONE_ARDRONE2_CONSTANTS_H
#define LIBDRONE_ARDRONE2_CONSTANTS_H

namespace ardrone2
{
	const std::string DEFAULT_IP   = "192.168.1.1";
	const int NAVDATA_PORT         = 5554;
	const int VIDEO_PORT           = 5555;
	const int VIDEO_RECORDING_PORT = 5553;
	const int CONTROL_PORT         = 5556;

	namespace navdata_keys
	{
		const int NAVDATA_DEMO_TAG = 0;
		const int NAVDATA_TIME_TAG = 1;
		const int NAVDATA_RAW_MEASURES_TAG = 2;
		const int NAVDATA_PHYS_MEASURES_TAG = 3;
		const int NAVDATA_GYROS_OFFSETS_TAG = 4;
		const int NAVDATA_EULER_ANGLES_TAG = 5;
		const int NAVDATA_REFERENCES_TAG = 6;
		const int NAVDATA_TRIMS_TAG = 7;
		const int NAVDATA_RC_REFERENCES_TAG = 8;
		const int NAVDATA_PWM_TAG = 9;
		const int NAVDATA_ALTITUDE_TAG = 10;
		const int NAVDATA_VISION_RAW_TAG = 11;
		const int NAVDATA_MAGNETO_TAG = 22;
		const int NAVDATA_WIFI_TAG = 26;
		const int NAVDATA_CKS_TAG = 0xFFFF;
	}

	namespace video
	{
		namespace frame_type
		{
			const int UNKNOWN = 0;
			const int IDR = 1;
			const int I = 2;
			const int P = 3;
			const int HEADERS = 4;
		}

		namespace codec
		{
			const int UNKNOWN = 0;
			const int VLIB = 1;
			const int P264 = 2;
			const int MPEG4_VISUAL = 3;
			const int MPEG4_AVC = 4;
		}
	}

	namespace app_id
	{
		const std::string SESSION_ID     = "af2af2af";
		const std::string PROFILE_ID     = "af1af1af";
		const std::string APPLICATION_ID = "af0af0af";
	}

	namespace flip
	{
		const int LEFT   = 18;
		const int RIGHT  = 19;
		const int AHEAD  = 16;
		const int BEHIND = 17;
	}

	namespace camera
	{
		const int FRONT  = 1;
		const int BOTTOM = 0;
	}

	namespace config
	{
		const std::string ALTITUDE_MAX         = "control:altitude_max";
		const std::string I_ALTITUDE_MAX       = "control:indoor_altitude_max";
		const std::string O_ALTITUDE_MAX       = "control:outdoor_altitude_max";
		const std::string OUTDOOR_SHELL        = "control:flight_without_shell";
		const std::string OUTDOOR_FLIGHT       = "control:outdoor";
		const std::string TILT_MAX             = "control:euler_angle_max";
		const std::string I_TILT_MAX           = "control:indoor_euler_angle_max";
		const std::string O_TILT_MAX           = "control:outdoor_euler_angle_max";
		const std::string VERTICAL_SPEED_MAX   = "control:control_vz_max";
		const std::string I_VERTICAL_SPEED_MAX = "control:indoor_control_vz_max";
		const std::string O_VERTICAL_SPEED_MAX = "control:outdoor_control_vz_max";
		const std::string YAW_SPEED_MAX        = "control:control_yaw";
		const std::string I_YAW_SPEED_MAX      = "control:indoor_control_yaw";
		const std::string O_YAW_SPEED_MAX      = "control:outdoor_control_yaw";

		const std::string VIDEO_CODEC          = "video:video_codec";
		const std::string MAX_BITRATE		   = "video:max_bitrate";

		namespace codec
		{
			const int MP4_360P = 0x80;
			const int H264_360P = 0x81;
			const int MP4_360P_H264_720P = 0x82;
			const int H264_720P = 0x83;
			const int MP4_360P_SLRS = 0x84;
			const int H264_360P_SLRS = 0x85;
			const int H264_720P_SLRS = 0x86;
			const int H264_AUTO_RESIZE = 0x87;
			const int MP4_360P_H264_360P = 0x88;
		}
	}

	namespace ctrlstate {
	  const uint32_t
	  ARDRONE_FLY_MASK            = 1U << 0,  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
	  ARDRONE_VIDEO_MASK          = 1U << 1,  /*!< VIDEO MASK : (0) video disable, (1) video enable */
	  ARDRONE_VISION_MASK         = 1U << 2,  /*!< VISION MASK : (0) vision disable, (1) vision enable */
	  ARDRONE_CONTROL_MASK        = 1U << 3,  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
	  ARDRONE_ALTITUDE_MASK       = 1U << 4,  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
	  ARDRONE_USER_FEEDBACK_START = 1U << 5,  /*!< USER feedback : Start button state */
	  ARDRONE_COMMAND_MASK        = 1U << 6,  /*!< Control command ACK : (0) None, (1) one received */
	  ARDRONE_CAMERA_MASK         = 1U << 7,  /*!< CAMERA MASK : (0) camera not ready, (1) Camera ready */
	  ARDRONE_TRAVELLING_MASK     = 1U << 8,  /*!< Travelling mask : (0) disable, (1) enable */
	  ARDRONE_USB_MASK            = 1U << 9,  /*!< USB key : (0) usb key not ready, (1) usb key ready */
	  ARDRONE_NAVDATA_DEMO_MASK   = 1U << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
	  ARDRONE_NAVDATA_BOOTSTRAP   = 1U << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
	  ARDRONE_MOTORS_MASK         = 1U << 12, /*!< Motors status : (0) Ok, (1) Motors problem */
	  ARDRONE_COM_LOST_MASK       = 1U << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
	  ARDRONE_SOFTWARE_FAULT      = 1U << 14, /*!< Software fault detected - user should land as quick as possible (1) */
	  ARDRONE_VBAT_LOW            = 1U << 15, /*!< VBat low : (1) too low, (0) Ok */
	  ARDRONE_USER_EL             = 1U << 16, /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
	  ARDRONE_TIMER_ELAPSED       = 1U << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
	  ARDRONE_MAGNETO_NEEDS_CALIB = 1U << 18, /*!< Magnetometer calibration state : (0) Ok, no calibration needed, (1) not ok, calibration needed */
	  ARDRONE_ANGLES_OUT_OF_RANGE = 1U << 19, /*!< Angles : (0) Ok, (1) out of range */
	  ARDRONE_WIND_MASK           = 1U << 20, /*!< WIND MASK: (0) ok, (1) Too much wind */
	  ARDRONE_ULTRASOUND_MASK     = 1U << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
	  ARDRONE_CUTOUT_MASK         = 1U << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
	  ARDRONE_PIC_VERSION_MASK    = 1U << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
	  ARDRONE_ATCODEC_THREAD_ON   = 1U << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
	  ARDRONE_NAVDATA_THREAD_ON   = 1U << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
	  ARDRONE_VIDEO_THREAD_ON     = 1U << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
	  ARDRONE_ACQ_THREAD_ON       = 1U << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
	  ARDRONE_CTRL_WATCHDOG_MASK  = 1U << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
	  ARDRONE_ADC_WATCHDOG_MASK   = 1U << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
	  ARDRONE_COM_WATCHDOG_MASK   = 1U << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
	  ARDRONE_EMERGENCY_MASK      = 1U << 31; /*!< Emergency landing : (0) no emergency, (1) emergency */
	}
}

#endif
