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

using namespace std;

ARDrone2::ARDrone2() : ARDrone2(ardrone2::DEFAULT_IP) {}

ARDrone2::ARDrone2(string ip)
{
	_ip = ip;
}

void ARDrone2::setIP(string ip)
{
	_ip = ip;
}

drone::limits ARDrone2::getLimits()
{
    //TODO: this
    drone::limits limits;
    
    return limits;
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
	navdata->attitude = Eigen::Vector3f(ardrone2_navdata->theta, ardrone2_navdata->psi, ardrone2_navdata->phi);
	navdata->batteryStatus = ardrone2_navdata->vbatpercentage / 100;
	navdata->flying = ardrone2_navdata->ctrlstate & ardrone2::ctrlstate::ARDRONE_FLY_MASK;
	navdata->linearvelocity = Eigen::Vector3f(ardrone2_navdata->vx, ardrone2_navdata->vy, ardrone2_navdata->vz);
	navdata->linkQuality = ardrone2_navdata->wifipercentage;

	static_pointer_cast<ardrone2::navdata>(navdata)->acceleration = Eigen::Vector3f(ardrone2_navdata->ax, ardrone2_navdata->ay, ardrone2_navdata->az);
	static_pointer_cast<ardrone2::navdata>(navdata)->magnetometer = Eigen::Vector3f(ardrone2_navdata->mx, ardrone2_navdata->my, ardrone2_navdata->mz);
	static_pointer_cast<ardrone2::navdata>(navdata)->pressure = ardrone2_navdata->p;

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

            //TODO: convert to value between -1 and 1
            _latestAttitudeCommand = AttitudeCommand(phi, theta, gaz, yaw);

    		_commandqueue.push_back(_latestAttitudeCommand);
		}
		break;
	case drone::commands::id::EMERGENCY:
	    {
	        _commandqueue.push_back(EmergencyCommand(true));
	    }
	    break;
	case drone::commands::id::FTTRIM:
	    {
	        _commandqueue.push_back(FlatTrimCommand());
	    }
	    break;
	case drone::commands::id::LAND:
	    {
	        _commandqueue.push_back(LandCommand());
	    }
	    break;
	case drone::commands::id::TAKEOFF:
	    {
	        _commandqueue.push_back(TakeOffCommand());
	    }
	    break;
	// AR.Drone 2.0 specific commands:  
	case ardrone2::commands::id::MAGNETOCALIB:
	    {
	        _commandqueue.push_back(MagnetometerCalibrationCommand());
	    }
	    break;
	case ardrone2::commands::id::CONFIG:
	    {
	        string key = boost::any_cast<string>(command.parameters[0]);
	        string value = boost::any_cast<string>(command.parameters[1]);
	    
	        _commandqueue.push_back(ConfigCommand(key, value));
	    }
	    break;
	case ardrone2::commands::id::RECORDONUSB:
	    {
	        bool record = boost::any_cast<bool>(command.parameters[0]);
	    
	        _commandqueue.push_back(RecordOnUSBCommand(record));
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
	        _commandqueue.push_back(ZapCommand(front));
	    }
	    break;
	}
	return true;
}

bool ARDrone2::processNoCommand()
{
	_commandqueue.push_back(_latestAttitudeCommand);
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
