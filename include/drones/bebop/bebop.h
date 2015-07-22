#ifndef LIBDRONE_BEBOP_H
#define LIBDRONE_BEBOP_H

#include <drone.h>
#include <drones/fpvdrone.h>
#include <types.h>
#include <drones/bebop/constants.h>
#include <drones/fpvtypes.h>

#include "../src/drones/bebop/controllink.h"
#include "../src/drones/bebop/fullnavdata.h"

#include <boost/asio.hpp>

#include <string>
#include <memory>
#include <limits>
#include <cmath>

class Bebop : public FPVDrone
{
	public:
		explicit Bebop(std::string ip);
		Bebop();

		void setIP(std::string ip);
		std::string getIP();

		drone::limits getLimits();
		drone::config getConfig();
		void setLimits(drone::limits limits);
		void setConfig(drone::config config);

		fpvdrone::picturestatus takePicture();
		bool isRecording();
		fpvdrone::picturestatus startRecording();
		void stopRecording();

	protected:
		drone::connectionstatus tryConnecting();
		void beforeUpdate();
		void updateCycle();
		bool decodeNavdata(std::shared_ptr<drone::navdata> &navdata);
		bool decodeVideo(cv::Mat &frame);
		bool processCommand(drone::command &command);
		bool processNoCommand();
		void connectionLost();

	private:
		std::string _ip;
		std::unique_ptr<bebop::controllink> _ctrllink = nullptr;
		std::unique_ptr<bebop::fullnavdata> _fullnavdata = nullptr;
		std::unique_ptr<boost::asio::io_service> _io_service = nullptr;
		drone::command _lastAttitudeCommand = drone::commands::attitude();

		drone::config _defaultConfig{drone::limits{0.2f, 2.0f, 1.2f, 5.0f}, false, true};
		drone::config _customInitialConfig{drone::limits{0, 0, 0, 0}, false, false};

		bool _outdoor = false;

		bool _recording = false;

		bool _config_initialized = false;
};

#endif
