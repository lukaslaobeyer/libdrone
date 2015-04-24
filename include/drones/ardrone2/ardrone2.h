#ifndef LIBDRONE_ARDRONE2_H
#define LIBDRONE_ARDRONE2_H

#include <drone.h>
#include <drones/fpvdrone.h>
#include <types.h>
#include <drones/ardrone2/constants.h>
#include <drones/ardrone2/types.h>
#include <drones/ardrone2/commands.h>

#include "../src/drones/ardrone2/controllink.h"
#include "../src/drones/ardrone2/navdata/navdatamanager.h"
#include "../src/drones/ardrone2/video/videomanager.h"

#include "../src/drones/ardrone2/atcommands/attitudecommand.h"

#include <string>
#include <stdexcept>

#include <boost/asio.hpp>

class ARDrone2 : public FPVDrone
{
	public:
		explicit ARDrone2(std::string ip);
		ARDrone2();

		void setIP(std::string ip);

		drone::limits getLimits();

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
		boost::asio::io_service *_io_service = nullptr;

		ControlLink _cl;
		NavdataManager _nm;
		VideoManager _vm;

		int _default_codec = ardrone2::config::codec::H264_360P;

		std::vector<ATCommand> _commandqueue;

		AttitudeCommand _latestAttitudeCommand;

		bool _flying = false;
};

#endif
