#include <drones/ardrone2/ardrone2.h>

#include "atcommands/configcommand.h"
#include "atcommands/configidscommand.h"
#include "atcommands/controlcommand.h"
#include "atcommands/attitudecommand.h"
#include "atcommands/emergencycommand.h"
#include "atcommands/flattrimcommand.h"
#include "atcommands/flipcommand.h"
#include "atcommands/hovercommand.h"
#include "atcommands/landcommand.h"
#include "atcommands/magnetometercalibrationcommand.h"
#include "atcommands/recordonusbcommand.h"
#include "atcommands/resetwatchdogcommand.h"
#include "atcommands/takeoffcommand.h"
#include "atcommands/zapcommand.h"

#include <iostream>
#include <limits>
#include <cmath>

#include <boost/filesystem.hpp>

using namespace std;

ARDrone2::ARDrone2(string saveDir) : ARDrone2(saveDir, ardrone2::DEFAULT_IP) {}

ARDrone2::ARDrone2(string saveDir, string ip) : _saveDir(saveDir), _ip(ip) {}

void ARDrone2::setIP(string ip)
{
	_ip = ip;
}

drone::limits ARDrone2::getLimits()
{
	if(isnan(_currentLimits.altitude) || isnan(_currentLimits.angle) || isnan(_currentLimits.vspeed) || isnan(_currentLimits.yawspeed))
	{
		return _defaultLimits;
	}
	else
	{
		return _currentLimits;
	}
}

drone::config ARDrone2::getConfig()
{
	drone::config config;

	config.limits = getLimits();
	config.outdoor = _outdoor;
	config.valid = true;

	return config;
}

void ARDrone2::setLimits(drone::limits limits)
{
	//TODO: De-uglify this

	_currentLimits = limits;

	string config_tilt = to_string((float) M_PI/2.0f);
	string max_altitude = to_string(limits.altitude);
	string max_angle = to_string(limits.angle);
	string max_vspeed = to_string(limits.vspeed);
	string max_yawspeed = to_string(limits.yawspeed);

	// Decimal separator needs to be a dot (to_string seems to recognize the current locale)
	replace(config_tilt.begin(), config_tilt.end(), ',', '.');
	replace(max_altitude.begin(), max_altitude.end(), ',', '.');
	replace(max_angle.begin(), max_angle.end(), ',', '.');
	replace(max_vspeed.begin(), max_vspeed.end(), ',', '.');
	replace(max_yawspeed.begin(), max_yawspeed.end(), ',', '.');

	vector<ATCommand> configCommands {
		ConfigIDSCommand(), ConfigCommand("control:control_iphone_tilt", config_tilt),
		ConfigIDSCommand(), ConfigCommand("control:altitude_max", max_altitude),
		ConfigIDSCommand(), ConfigCommand("control:euler_angle_max", max_angle),
		ConfigIDSCommand(), ConfigCommand("control:control_vz_max", max_vspeed),
		ConfigIDSCommand(), ConfigCommand("control:control_yaw", max_yawspeed)
	};

	_cmdmutex.lock();
	_commandqueue.insert(_commandqueue.end(), configCommands.begin(), configCommands.end());
	_cmdmutex.unlock();
}

void ARDrone2::setConfig(drone::config config)
{
	if(config.valid)
	{
		setLimits(config.limits);

		vector<ATCommand> configCommands {
			ConfigIDSCommand()
		};

		_outdoor = config.outdoor;

		if(config.outdoor)
		{
			configCommands.push_back(ConfigCommand("control:outdoor", "TRUE"));
			configCommands.push_back(ConfigIDSCommand());
			configCommands.push_back(ConfigCommand("control:flight_without_shell", "TRUE"));
		}
		else
		{
			configCommands.push_back(ConfigCommand("control:outdoor", "FALSE"));
			configCommands.push_back(ConfigIDSCommand());
			configCommands.push_back(ConfigCommand("control:flight_without_shell", "FALSE"));
		}

		_cmdmutex.lock();
		_commandqueue.insert(_commandqueue.end(), configCommands.begin(), configCommands.end());
		_cmdmutex.unlock();
	}
}

cv::Mat ARDrone2::getLatestFrame()
{
	return _vm.getVideoFrame();
}

unsigned long ARDrone2::getFrameAge()
{
	unsigned long age = (std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1)) - _vm.getLastFrameTime();
	return age;
}

fpvdrone::picturestatus ARDrone2::takePicture()
{
	try
	{
		string picdirectory = _saveDir;
		picdirectory.append("Pictures/");

		// Create the directory if it doesn't exist
		boost::filesystem::create_directories(picdirectory);

		const boost::posix_time::ptime time = boost::posix_time::second_clock::local_time();
		stringstream timestamp;
		timestamp << setw(4) << setfill('0') << time.date().year() << setw(2) << time.date().month().as_number() << setw(2) << time.date().day().as_number();
		timestamp << "T";
		timestamp << setw(2) << time.time_of_day().hours() << setw(2) << time.time_of_day().minutes() << setw(2) << time.time_of_day().seconds();

		string filename = "Pic_";
		filename.append(timestamp.str());
		filename.append(".jpg");

		string path = picdirectory + filename;
		if(_vm.takePicture(path))
		{
			return fpvdrone::PIC_OK;
		}
		else
		{
			return fpvdrone::PIC_ERROR;
		}
	}
	catch(std::exception &ex)
	{
		cerr << "Error! Could not save picture." << endl;
		cerr << ex.what() << endl;
		return fpvdrone::PIC_ERROR;
	}

	return fpvdrone::PIC_ERROR;
}

bool ARDrone2::isRecording()
{
	return _recording;
}

fpvdrone::picturestatus ARDrone2::startRecording()
{
	try
	{
		// Create the dirextory if it doesn't exist
		string videodirectory = _saveDir;
		videodirectory.append("Videos/");
		boost::filesystem::create_directories(videodirectory);

		const boost::posix_time::ptime time = boost::posix_time::second_clock::local_time();
		stringstream timestamp;
		timestamp << setw(4) << setfill('0') << time.date().year() << setw(2) << time.date().month().as_number() << setw(2) << time.date().day().as_number();
		timestamp << "T";
		timestamp << setw(2) << time.time_of_day().hours() << setw(2) << time.time_of_day().minutes() << setw(2) << time.time_of_day().seconds();

		string filename = "Vid_";
		filename.append(timestamp.str());
		filename.append(".mp4");

		if(_vm.startRecording(videodirectory + filename))
		{
			_recording = true;
			return fpvdrone::PIC_OK;
		}
		else
		{
			return fpvdrone::PIC_ERROR;
		}
	}
	catch(runtime_error &e)
	{
		cerr << "Error: " << e.what() << endl;
		return fpvdrone::PIC_ERROR;
	}

	return fpvdrone::PIC_ERROR;
}

void ARDrone2::stopRecording()
{
	_vm.stopRecording();
	_recording = false;
}

drone::error ARDrone2::accept(dronevisitor *visitor)
{
	return visitor->visit(shared_from_this());
}

drone::connectionstatus ARDrone2::tryConnecting()
{
	try
	{
		if(_io_service == nullptr)
		{
			_io_service = new boost::asio::io_service;
		}

		// Initialize communication with AR.Drone
		_cl.init(_ip, *_io_service);


		// Needed for the AR.Drone to send full navigation data and accept commands (Somewhat like described in the dev guide in section 7.1.2, and some magic)
		_cl.setAppID();
		boost::this_thread::sleep_for(boost::chrono::milliseconds(250)); // Wait until the drone has performed its configuration switch (important)
		_cl.sendATCommands(vector<ATCommand>{ConfigIDSCommand(), ConfigCommand("general:navdata_demo", "TRUE"), ControlCommand(0)});
		boost::this_thread::sleep_for(boost::chrono::milliseconds(250));
		_cl.sendATCommands(vector<ATCommand>{ConfigIDSCommand(), ConfigCommand("general:navdata_demo", "FALSE"), ConfigIDSCommand(), ConfigCommand("general:navdata_options", "268435455"), ControlCommand(5)});
		boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
		_cl.sendATCommands(vector<ATCommand>{ConfigIDSCommand(), ConfigCommand("control:control_level", ATCommand::_int(1<<0))});

		// Wait for the AR.Drone to process the commands
		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

		_cl.sendATCommands(vector<ATCommand>{ConfigIDSCommand(), ConfigCommand(ardrone2::config::VIDEO_CODEC, to_string(_default_codec)), ConfigIDSCommand(), ConfigCommand(ardrone2::config::MAX_BITRATE, to_string(4000))});

		// Init navdata manager
		_nm.init(_ip, *_io_service);

		// Wait for navdata packets to be received
		boost::this_thread::sleep_for(boost::chrono::milliseconds(250));

		// Process received packets (if any)
		_nm.update();

		// Check that navdata has indeed been received
		bool connected = _nm.isConnected();

		if(connected)
		{
			_vm.init(_ip, *_io_service);
			_vm.update();

			return drone::connectionstatus::CONNECTION_ESTABLISHED;
		}
		else
		{
			_cl.close();
			_nm.close();
			return drone::connectionstatus::CONNECTION_FAILED;
		}
	}
	catch(const boost::system::system_error &e)
	{
		cerr << "Error: " << e.what() << endl;
		if(e.code().value() != 1)
		{
			try
			{
				_cl.close();
			}
			catch(const boost::system::system_error &e)
			{}
		}
		return drone::connectionstatus::EXCEPTION_OCCURRED;
	}
}

void ARDrone2::beforeUpdate()
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
			cout << "[WARNING] Lost connection to AR.Drone!" << endl;

			markConnectionLost();
		}
	}
	else
	{
		zero_packets_counter = 0;
	}
}

void ARDrone2::updateCycle()
{
	FPVDrone::updateCycle();

	_cl.sendATCommands(_commandqueue);
	_commandqueue.clear();
}

bool ARDrone2::decodeNavdata(shared_ptr<drone::navdata> &navdata)
{
	uint32_t prevSeqNum = 0;

	AFNavdata *ardrone2_navdata = _nm.getNavdata();

	if(prevSeqNum >= ardrone2_navdata->n)
	{
		return false;
	}

	prevSeqNum = ardrone2_navdata->n;

	navdata = make_shared<ardrone2::navdata>();

	navdata->altitude = ardrone2_navdata->altitude;
	navdata->attitude = Eigen::Vector3f(ardrone2_navdata->theta / (180.0f/M_PI), ardrone2_navdata->phi / (180.0f/M_PI), ardrone2_navdata->psi / (180.0f/M_PI));
	navdata->batterystatus = ardrone2_navdata->vbatpercentage / 100.0f;
	navdata->linearvelocity = Eigen::Vector3f(ardrone2_navdata->vx, ardrone2_navdata->vy, ardrone2_navdata->vz);
	navdata->linkquality = -1; // AR.Drone 2.0 WiFi quality indication does for whatever reason not work

	static_pointer_cast<ardrone2::navdata>(navdata)->acceleration = Eigen::Vector3f(ardrone2_navdata->ax, ardrone2_navdata->ay, ardrone2_navdata->az);
	static_pointer_cast<ardrone2::navdata>(navdata)->magnetometer = Eigen::Vector3f(ardrone2_navdata->mx, ardrone2_navdata->my, ardrone2_navdata->mz);
	static_pointer_cast<ardrone2::navdata>(navdata)->pressure = ardrone2_navdata->p;

	std::bitset<32> ctrlstate(ardrone2_navdata->ctrlstate);
	if(ctrlstate.test(0))
	{
		_flying = true; // This bit is set after taking off, but is unset when moving the drone (What's going on, Parrot?)
					    // The _flying variable gets set to false when a landing command is requested.
	}
	navdata->flying = _flying;

	return true;
}

bool ARDrone2::processCommand(drone::command &command)
{
	// TODO: Convert standard command to ATCommand and push back to command queue
	switch(command.command)
	{
	case drone::commands::id::ATTITUDE:
		{
    		Eigen::Vector3f attitude = boost::any_cast<Eigen::Vector3f>(command.parameters[0]);
    		float vspeed = boost::any_cast<float>(command.parameters[1]);

            drone::limits limits = getLimits();

            float theta = applyLimit(attitude(0), limits.angle); // pitch
            float phi = applyLimit(attitude(1), limits.angle); // roll
            float yaw = applyLimit(attitude(2), limits.yawspeed); // yawspeed
            float gaz = applyLimit(vspeed, limits.vspeed); // vspeed

            // Convert from angle to value between -1.0 and 1.0
            theta = theta * (1/limits.angle);
            phi = phi * (1/limits.angle);
            yaw = yaw * (1/limits.yawspeed);
            gaz = gaz * (1/limits.vspeed);

            _latestAttitudeCommand = AttitudeCommand(phi, theta, gaz, yaw);

            _cmdmutex.lock();
            _commandqueue.push_back(_latestAttitudeCommand);
            _cmdmutex.unlock();
		}
		break;
	case drone::commands::id::ATTITUDEREL:
		{
			Eigen::Vector3f attitude = boost::any_cast<Eigen::Vector3f>(command.parameters[0]);
			float vspeed = boost::any_cast<float>(command.parameters[1]);

			drone::limits limits = getLimits();

			float theta = applyLimit(attitude(0), 1.0f); // pitch
			float phi = applyLimit(attitude(1), 1.0f); // roll
			float yaw = applyLimit(attitude(2), 1.0f); // yawspeed
			float gaz = applyLimit(vspeed, 1.0f); // vspeed

			_latestAttitudeCommand = AttitudeCommand(phi, theta, gaz, yaw);

			_cmdmutex.lock();
			_commandqueue.push_back(_latestAttitudeCommand);
			_cmdmutex.unlock();
		}
		break;
	case drone::commands::id::EMERGENCY:
	    {
	    	_cmdmutex.lock();
	    	_commandqueue.push_back(EmergencyCommand(true));
	    	_cmdmutex.unlock();
	    }
	    break;
	case drone::commands::id::FTTRIM:
	    {
	    	_cmdmutex.lock();
	    	_commandqueue.push_back(FlatTrimCommand());
	    	_cmdmutex.unlock();
	    }
	    break;
	case drone::commands::id::LAND:
	    {
	    	_cmdmutex.lock();
	    	_commandqueue.push_back(LandCommand());
	    	_cmdmutex.unlock();
			_flying = false;
	    }
	    break;
	case drone::commands::id::TAKEOFF:
	    {
	    	_cmdmutex.lock();
	    	_commandqueue.push_back(TakeOffCommand());
	    	_cmdmutex.unlock();
	    }
	    break;
	// AR.Drone 2.0 specific commands:
	case ardrone2::commands::id::MAGNETOCALIB:
	    {
	    	_cmdmutex.lock();
	    	_commandqueue.push_back(MagnetometerCalibrationCommand());
	    	_cmdmutex.unlock();
	    }
	    break;
	case ardrone2::commands::id::CONFIG:
	    {
	        string key = boost::any_cast<string>(command.parameters[0]);
	        string value = boost::any_cast<string>(command.parameters[1]);

	        _cmdmutex.lock();
	        _commandqueue.push_back(ConfigCommand(key, value));
	        _cmdmutex.unlock();
	    }
	    break;
	case ardrone2::commands::id::RECORDONUSB:
	    {
	        bool record = boost::any_cast<bool>(command.parameters[0]);

	        _cmdmutex.lock();
	        _commandqueue.push_back(RecordOnUSBCommand(record));
	        _cmdmutex.unlock();
	    }
	    break;
	case ardrone2::commands::id::SWITCHVIEW:
	    {
	        ardrone2::commands::switchview::view view = boost::any_cast<ardrone2::commands::switchview::view>(command.parameters[0]);
	        bool front = false;
	        if(view == ardrone2::commands::switchview::view::FRONT)
	        {
	            front = true;
	        }

	        _cmdmutex.lock();
	        _commandqueue.push_back(ZapCommand(front));
	        _cmdmutex.unlock();
	    }
	    break;
	case ardrone2::commands::id::FLIP:
		{
			ardrone2::commands::flip::direction direction = boost::any_cast<ardrone2::commands::flip::direction>(command.parameters[0]);
			int dir = 0;
			switch(direction)
			{
			case ardrone2::commands::flip::direction::FRONT:
				dir = 0;
				break;
			case ardrone2::commands::flip::direction::LEFT:
				dir = 1;
				break;
			case ardrone2::commands::flip::direction::BACK:
				dir = 2;
				break;
			case ardrone2::commands::flip::direction::RIGHT:
				dir = 3;
				break;
			}

			_cmdmutex.lock();
			_commandqueue.push_back(FlipCommand(dir));
			_cmdmutex.unlock();
		}
		break;
	}
	return true;
}

bool ARDrone2::processNoCommand()
{
	_cmdmutex.lock();
	_commandqueue.push_back(_latestAttitudeCommand);
	_cmdmutex.unlock();
	return true;
}

bool ARDrone2::decodeVideo(cv::Mat &frame)
{
	static unsigned long lastDecodedFrame = 0;

	if(_vm.decodedPackets() == lastDecodedFrame)
	{
		return false;
	}

	lastDecodedFrame = _vm.decodedPackets();

	frame = _vm.getVideoFrame();

	if(frame.empty())
	{
		return false;
	}

	return true;
}

void ARDrone2::connectionLost()
{
	// Reset state to unconnected
	_vm.stopRecording();

	_vm.close();
	_nm.close();
	_cl.close();

	_io_service->stop();
	delete _io_service;
	_io_service = nullptr;
}
