#include <drones/bebop/bebop.h>

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
			_io_service = new boost::asio::io_service;
		}

		bebop::controllink ctrllink;
		ctrllink.init(_ip, *_io_service);
	}
	catch(const boost::system::system_error &e)
	{
		cerr << "Error: " << e.what() << endl;

		return drone::connectionstatus::EXCEPTION_OCCURRED;
	}
}

void Bebop::beforeUpdate()
{

}

void Bebop::updateCycle()
{

}

bool Bebop::decodeNavdata(std::shared_ptr<drone::navdata> &navdata)
{

}

bool Bebop::decodeVideo(cv::Mat &frame)
{

}

bool Bebop::processCommand(drone::command &command)
{

}

bool Bebop::processNoCommand()
{

}

void Bebop::connectionLost()
{

}
