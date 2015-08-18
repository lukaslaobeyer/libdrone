#include <drones/bebop/bebop.h>
#include <drones/bebop/commands.h>

#include <limits>

#include <boost/log/trivial.hpp>

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

string Bebop::getIP()
{
	return _ip;
}

drone::limits Bebop::getLimits()
{
	if(_ctrllink == nullptr)
	{
		return _defaultConfig.limits;
	}

	drone::limits currentLimits = _ctrllink->getLimits();

	if(isnan(currentLimits.altitude) || isnan(currentLimits.angle) || isnan(currentLimits.vspeed) || isnan(currentLimits.yawspeed))
	{
		return _defaultConfig.limits;
	}
	else
	{
		return currentLimits;
	}
}

drone::config Bebop::getConfig()
{
	drone::config config;
	config.limits = getLimits();
	config.outdoor = _outdoor;
	config.valid = true;
	return config;
}

void Bebop::setLimits(drone::limits limits)
{
	if(_ctrllink == nullptr)
	{
		return;
	}

	_ctrllink->setLimits(limits.altitude, limits.angle, limits.vspeed, limits.yawspeed);
}

void Bebop::setConfig(drone::config config)
{
	if(config.valid)
	{
		if(_ctrllink == nullptr)
		{
			_customInitialConfig = config;
			_outdoor = config.outdoor;
		}
		else
		{
			_ctrllink->setConfig(config);
			_outdoor = config.outdoor;
		}
	}
}

void Bebop::setVideoSettings(bebop::pictureformat pic_fmt, bebop::whitebalancemode wb_mode, bebop::antiflickermode af_mode, float exposure, float saturation)
{
	bebop::navdata_id picfmt_id = bebop::command_ids::picture_format;
	vector<boost::any> picfmt_args{(uint32_t) pic_fmt};

	bebop::navdata_id wbmode_id = bebop::command_ids::whitebalance_mode;
	vector<boost::any> wbmode_args{(uint32_t) wb_mode};

	bebop::navdata_id afmode_id = bebop::command_ids::antiflickering_mode;
	vector<boost::any> afmode_args{(uint32_t) af_mode};

	bebop::navdata_id exp_id = bebop::command_ids::picture_exposure;
	vector<boost::any> exp_args{(float) exposure};

	bebop::navdata_id sat_id = bebop::command_ids::picture_saturation;
	vector<boost::any> sat_args{(float) saturation};

	if(_ctrllink == nullptr)
	{
		_initialCommands_id_queue.push(picfmt_id);
		_initialCommands_arg_queue.push(picfmt_args);

		_initialCommands_id_queue.push(wbmode_id);
		_initialCommands_arg_queue.push(wbmode_args);

		_initialCommands_id_queue.push(afmode_id);
		_initialCommands_arg_queue.push(afmode_args);

		_initialCommands_id_queue.push(exp_id);
		_initialCommands_arg_queue.push(exp_args);

		_initialCommands_id_queue.push(sat_id);
		_initialCommands_arg_queue.push(sat_args);
	}
	else
	{
		_ctrllink->sendCommand(picfmt_id, picfmt_args);
		_ctrllink->sendCommand(wbmode_id, wbmode_args);
		_ctrllink->sendCommand(afmode_id, afmode_args);
		_ctrllink->sendCommand(exp_id, exp_args);
		_ctrllink->sendCommand(sat_id, sat_args);
	}
}

void Bebop::resetSettings()
{
	bebop::navdata_id reset_id = bebop::command_ids::reset;
	vector<boost::any> reset_args{};

	if(_ctrllink == nullptr)
	{
		_initialCommands_id_queue.push(reset_id);
		_initialCommands_arg_queue.push(reset_args);
	}
	else
	{
		_ctrllink->sendCommand(reset_id, reset_args);
	}
}

cv::Mat Bebop::getLatestFrame()
{
	if(_ctrllink == nullptr)
	{
		return cv::Mat();
	}

	return _ctrllink->getVideoFrame();
}

unsigned long Bebop::getFrameAge()
{
	if(_ctrllink == nullptr)
	{
		return std::numeric_limits<unsigned long>::max();
	}

	unsigned long age = (std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1)) - _ctrllink->getLatestFrameTime();
	return age;
}

fpvdrone::picturestatus Bebop::takePicture()
{
	if(_ctrllink == nullptr)
	{
		return fpvdrone::PIC_ERROR;
	}

	if(!_ctrllink->isReadyForTakingPicture())
	{
		return fpvdrone::PIC_BUSY;
	}

	if(_recording)
	{
		return fpvdrone::PIC_BUSY;
	}

	bebop::navdata_id command_id_0 = bebop::command_ids::video_autorecord;
	vector<boost::any> args_0{(uint8_t) 0, (uint8_t) 0};

	_ctrllink->sendCommand(command_id_0, args_0);

	bebop::navdata_id command_id = bebop::command_ids::take_picture;
	vector<boost::any> args{};

	_ctrllink->sendCommand(command_id, args);

	return fpvdrone::PIC_OK;
}

bool Bebop::isRecording()
{
	return _recording;
}

fpvdrone::picturestatus Bebop::startRecording()
{
	if(_ctrllink == nullptr)
	{
		return fpvdrone::PIC_ERROR;
	}

	bebop::navdata_id command_id_0 = bebop::command_ids::video_autorecord;
	vector<boost::any> args_0{(uint8_t) 1, (uint8_t) 0};

	_ctrllink->sendCommand(command_id_0, args_0);

	bebop::navdata_id command_id = bebop::command_ids::video;
	vector<boost::any> args{(uint8_t) 1, (uint8_t) 0, (uint8_t) 0, (uint8_t) 0};

	_ctrllink->sendCommand(command_id, args);

	_recording = true;

	return fpvdrone::PIC_OK;
}

void Bebop::stopRecording()
{
	if(_ctrllink == nullptr)
	{
		return;
	}

	bebop::navdata_id command_id = bebop::command_ids::video;
	vector<boost::any> args{(uint8_t) 0, (uint8_t) 0, (uint8_t) 0, (uint8_t) 0};

	_ctrllink->sendCommand(command_id, args);

	_recording = false;
}

drone::error Bebop::accept(dronevisitor *visitor)
{
	return visitor->visit(shared_from_this());
}

drone::connectionstatus Bebop::tryConnecting()
{
	try
	{
		_io_service.reset(new boost::asio::io_service);

		_ctrllink.reset(new bebop::controllink);
		_ctrllink->init(_ip, *_io_service);

		_fullnavdata.reset(new bebop::fullnavdata);
		try
		{
			_fullnavdata->init(_ip, *_io_service);
		}
		catch(const boost::system::system_error &e)
		{
			BOOST_LOG_TRIVIAL(info) << "Full navdata not available.";
		}

		_config_initialized = false;

		while(_initialCommands_id_queue.size() > 0)
		{
			_ctrllink->sendCommand(_initialCommands_id_queue.front(), _initialCommands_arg_queue.front());
			_initialCommands_id_queue.pop();
			_initialCommands_arg_queue.pop();
		}

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

	// Initialize Bebop config on update loop start
	if(!_config_initialized)
	{
		_ctrllink->initConfig();
		if(_customInitialConfig.valid)
		{
			_ctrllink->setConfig(_customInitialConfig);
		}
		else
		{
			_ctrllink->setConfig(_defaultConfig);
		}
		_config_initialized = true;
	}

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
	_ctrllink->processCommandQueue();
}

bool Bebop::decodeNavdata(std::shared_ptr<drone::navdata> &navdata)
{
	if(_ctrllink == nullptr)
	{
		navdata = nullptr;
		return false;
	}

	shared_ptr<bebop::navdata> fullnavdata = _fullnavdata->getNavdata();
	if(fullnavdata != nullptr)
	{
		shared_ptr<bebop::navdata> standardnavdata = _ctrllink->getNavdata();
		fullnavdata->flying = standardnavdata->flying;
		fullnavdata->cameraorientation = standardnavdata->cameraorientation;
		fullnavdata->linkquality = standardnavdata->linkquality;
		fullnavdata->gps_fix = standardnavdata->gps_fix;
		navdata = fullnavdata;
	}
	else
	{
		navdata = _ctrllink->getNavdata();
	}

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
		try
		{
			_ctrllink->sendCommand(command_id, args);
		}
		catch(boost::system::system_error &ex)
		{
			BOOST_LOG_TRIVIAL(error) << "Connection lost! " << ex.what();
			return false;
		}
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
	_ctrllink->close();
	BOOST_LOG_TRIVIAL(debug) << "Control link closed";
}
