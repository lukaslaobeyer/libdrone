#ifndef LIBDRONE_BEBOP_H
#define LIBDRONE_BEBOP_H

#include <drone.h>
#include <drones/fpvdrone.h>
#include <types.h>

#include <boost/asio.hpp>

#include <string>

class Bebop : public FPVDrone
{
	public:
		explicit Bebop(std::string ip);
		Bebop();

		void setIP(std::string ip);

		drone::limits getLimits();

	protected:
		drone::connectionstatus tryConnecting();
		void beforeUpdate();
		void updateCycle();
		bool decodeNavdata(std::shared_ptr<drone::navdata> navdata);
		bool decodeVideo(cv::Mat &frame);
		bool processCommand(drone::command &command);
		bool processNoCommand();
		void connectionLost();

	private:
		std::string _ip;
		boost::asio::io_service *_io_service = nullptr;
};

#endif
