#include <drones/bebop/bebop.h>
#include <drones/bebop/commands.h>

#include "protocol.h"
#include "commandcomposer.h"

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

		_ctrllink.reset(new bebop::controllink);
		_ctrllink->init(_ip, *_io_service);

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
	FPVDrone::updateCycle();
}

bool Bebop::decodeNavdata(std::shared_ptr<drone::navdata> &navdata)
{
	navdata = _ctrllink->getNavdata();
	return true;
}

bool Bebop::decodeVideo(cv::Mat &frame)
{
	frame = _ctrllink->getVideoFrame();
	return true;
}

bool Bebop::processCommand(drone::command &command)
{
	bebop::navdata_id command_id;
	vector<boost::any> args;
	bool processed = false;

	switch(command.command)
	{
	case drone::commands::id::ATTITUDE:
		{
			Eigen::Vector3f attitude = boost::any_cast<Eigen::Vector3f>(command.parameters[0]);
			float vspeed = boost::any_cast<float>(command.parameters[1]);

			drone::limits limits = getLimits();

			float pitch = applyLimit(attitude(0), limits.angle); // pitch
			float roll = applyLimit(attitude(1), limits.angle); // roll
			float yaw = applyLimit(attitude(2), limits.yawspeed); // yawspeed
			float gaz = applyLimit(vspeed, limits.vspeed); // vspeed

			command_id = bebop::command_ids::pcmd;
			args = bebop::commands::create::pcmd(roll, -pitch, yaw, gaz, limits);

			_lastAttitudeCommand = command;

			processed = true;
		}
		break;
	case drone::commands::id::ATTITUDEREL:
		{
			Eigen::Vector3f attitude = boost::any_cast<Eigen::Vector3f>(command.parameters[0]);
			float vspeed = boost::any_cast<float>(command.parameters[1]);

			drone::limits limits = getLimits();

			float pitch = applyLimit(attitude(0), 1.0f); // pitch
			float roll = applyLimit(attitude(1), 1.0f); // roll
			float yaw = applyLimit(attitude(2), 1.0f); // yawspeed
			float gaz = applyLimit(vspeed, 1.0f); // vspeed

			command_id = bebop::command_ids::pcmd;
			args = bebop::commands::create::pcmdrel(roll, -pitch, yaw, gaz, limits);

			_lastAttitudeCommand = command;

			processed = true;
		}
		break;
	case drone::commands::id::TAKEOFF:
		{
			command_id = bebop::command_ids::takeoff;
			processed = true;
		}
		break;
	case drone::commands::id::LAND:
		{
			command_id = bebop::command_ids::land;
			processed = true;
		}
		break;
	case drone::commands::id::EMERGENCY:
		{
			command_id = bebop::command_ids::emergency;
			processed = true;
		}
		break;
	case drone::commands::id::FTTRIM:
		{
			command_id = bebop::command_ids::flattrim;
			processed = true;
		}
		break;
	// Bebop specific commands
	case bebop::commands::id::CAMERA_ORIENTATION:
		{
			float tilt = boost::any_cast<float>(command.parameters[0]);
			float pan = boost::any_cast<float>(command.parameters[1]);

			command_id = bebop::command_ids::camera_orientation;
			args = bebop::commands::create::camera_orientation(tilt, pan);
			processed = true;
		}
		break;
	case bebop::commands::id::FLIP:
		{
			bebop::commands::flip::direction direction = boost::any_cast<bebop::commands::flip::direction>(command.parameters[0]);

			command_id = bebop::command_ids::flip;
			args = bebop::commands::create::flip(direction);
			processed = true;
		}
		break;
	}

	if(processed)
	{
		_ctrllink->sendCommand(command_id, args);  //TODO: Handle exceptions?
		return true;
	}

	return false;
}

bool Bebop::processNoCommand()
{
	return processCommand(_lastAttitudeCommand);
}

void Bebop::connectionLost()
{

}
