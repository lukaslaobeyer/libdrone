#ifndef LIBDRONE_ARDRONE2_H
#define LIBDRONE_ARDRONE2_H

#include <drone.h>
#include <drones/fpvdrone.h>
#include <types.h>
#include <drones/ardrone2/constants.h>
#include <drones/ardrone2/types.h>
#include <drones/ardrone2/commands.h>
#include <drones/fpvtypes.h>

#include "../src/drones/ardrone2/controllink.h"
#include "../src/drones/ardrone2/navdata/navdatamanager.h"
#include "../src/drones/ardrone2/video/videomanager.h"

#include "../src/drones/ardrone2/atcommands/attitudecommand.h"

#include <string>
#include <stdexcept>
#include <mutex>

#include <boost/asio.hpp>

class ARDrone2 : public FPVDrone, public std::enable_shared_from_this<ARDrone2>
{
	public:
		explicit ARDrone2(std::string saveDir, std::string ip);
		ARDrone2(std::string saveDir);

		void setIP(std::string ip);

		drone::limits getLimits();
		drone::config getConfig();
		void setLimits(drone::limits limits);
		void setConfig(drone::config config);

		cv::Mat getLatestFrame();
		unsigned long getFrameAge();

		fpvdrone::picturestatus takePicture();
		bool isRecording();
		fpvdrone::picturestatus startRecording();
		void stopRecording();

		drone::error accept(dronevisitor *visitor);

	protected:
		drone::connectionstatus tryConnecting();
		void updateCycle();
		void beforeUpdate();
		bool decodeNavdata(std::shared_ptr<drone::navdata> &navdata);
		bool processCommand(drone::command &command);
		bool processNoCommand();
		bool decodeVideo(cv::Mat &frame);
		void connectionLost();

	private:
		std::string _ip;
		std::string _saveDir;
		boost::asio::io_service *_io_service = nullptr;

		ControlLink _cl;
		NavdataManager _nm;
		VideoManager _vm;

		int _default_codec = ardrone2::config::codec::H264_360P;

		std::mutex _cmdmutex;

		std::vector<ATCommand> _commandqueue;

		AttitudeCommand _latestAttitudeCommand;

		drone::limits _defaultLimits{0.2f, 2.0f, 1.2f, 5.0f};
		drone::limits _currentLimits{std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};

		bool _outdoor = false;

		bool _flying = false;
		bool _recording = false;
};

#endif
