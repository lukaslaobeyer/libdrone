#include <drones/bebop/bebop.h>

#include "protocol.h"

using namespace std;
using boost::asio::ip::tcp;

Bebop::Bebop() : Bebop(bebop::DEFAULT_IP) {}

Bebop::Bebop(string ip)
{
	_ip = ip;
}

void Bebop::setIP(string ip)
{
	_ip = ip;
}

drone::limits Bebop::getLimits()
{
	//TODO: this
	drone::limits limits;
	return limits;
}

drone::connectionstatus Bebop::tryConnecting()
{
	try
	{
		if(_io_service == nullptr)
		{
			_io_service.reset(new boost::asio::io_service);
		}

		ctrllink.reset(new bebop::controllink);
		ctrllink->init(_ip, *_io_service);

		return drone::connectionstatus::CONNECTION_ESTABLISHED;
	}
	catch(const boost::system::system_error &e)
	{
		cerr << "Error: " << e.what() << endl;

		return drone::connectionstatus::EXCEPTION_OCCURRED;
	}
}

void Bebop::beforeUpdate()
{
	static int zero_packets_counter = 0;

	// Poll data
	int packets = _io_service->poll();

	// Detect loss of connection
	if(packets == 0)
	{
		zero_packets_counter++;

		if(zero_packets_counter >= 50)
		{
			// Definitely lost connection
			cout << "[WARNING] Lost connection to Bebop!" << endl;

			markConnectionLost();
		}
	}
	else
	{
		zero_packets_counter = 0;
	}
}

void Bebop::updateCycle()
{

}

bool Bebop::decodeNavdata(std::shared_ptr<drone::navdata> &navdata)
{
	return false;
}

bool Bebop::decodeVideo(cv::Mat &frame)
{
	return false;
}

bool Bebop::processCommand(drone::command &command)
{
	bebop::navdata_id command_id;
	vector<boost::any> args;
	bool processed = false;

	switch(command.command)
	{
	case drone::commands::id::ATTITUDE:
		//TODO: this
		break;
	}

	if(processed)
	{
		ctrllink->sendCommand(command_id, args);  //TODO: Handle exceptions?
		return true;
	}

	return false;
}

bool Bebop::processNoCommand()
{
	drone::commands::attitude hover;
	return processCommand(hover);
}

void Bebop::connectionLost()
{

}
