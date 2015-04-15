#include <drones/bebop/bebop.h>

Bebop::Bebop(std::string ip)
{
	_ip = ip;
}

void Bebop::setIP(std::string ip)
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

}

void Bebop::beforeUpdate()
{

}

void Bebop::updateCycle()
{

}

bool Bebop::decodeNavdata(std::shared_ptr<drone::navdata> navdata)
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
