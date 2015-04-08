#include <drone.h>

#include <boost/timer/timer.hpp>

drone::connectionstatus Drone::connect()
{
	if(_connected.load())
	{
		return drone::connectionstatus::ALREADY_CONNECTED;
	}

	drone::connectionstatus status = tryConnecting();
	if(status == drone::connectionstatus::CONNECTION_ESTABLISHED)
	{
		_connected.store(true);
		notifyConnectionEstablished();
	}
	else
	{
		_connected.store(false);
	}
	return status;
}

bool Drone::isConnected()
{
	return _connected.load();
}

void Drone::addNavdataListener(INavdataListener *listener)
{
	_ndlisteners.push_back(listener);
}

void Drone::removeNavdataListener(INavdataListener *listener)
{
	_ndlisteners.erase(remove(_ndlisteners.begin(), _ndlisteners.end(), listener), _ndlisteners.end());
}

void Drone::addConnectionStatusListener(IConnectionStatusListener *listener)
{
	_cslisteners.push_back(listener);
}

void Drone::removeConnectionStatusListener(IConnectionStatusListener *listener)
{
	_cslisteners.erase(remove(_cslisteners.begin(), _cslisteners.end(), listener), _cslisteners.end());
}

void Drone::notifyNavdataListeners(std::shared_ptr<const drone::navdata> navdata)
{
	for(INavdataListener *i : _ndlisteners)
	{
		i->navdataAvailable(navdata);
	}
}

void Drone::notifyConnectionEstablished()
{
	for(IConnectionStatusListener *cslistener : _cslisteners)
	{
		cslistener->connectionEstablished();
	}
}

void Drone::notifyConnectionLost()
{
	for(IConnectionStatusListener *cslistener : _cslisteners)
	{
		cslistener->connectionLost();
	}
}

void Drone::markConnectionLost()
{
	_connected.store(false);
}

bool Drone::startUpdateLoop()
{
	if(_updater == nullptr)
	{
		if(_connected.load())
		{
			_stop_flag = false;
			_updater = new boost::thread(&Drone::runUpdateLoop, this);
		}
		else
		{
			return false;
		}
	}
	else if(_connected.load())
	{
		// When the connection is lost the _updater thread still exists. If it is reestablished, then it needs to be killed.

		stopUpdateLoop();
		_connected.store(true);
		startUpdateLoop();
	}

	return true;
}

void Drone::stopUpdateLoop()
{
	_stop_flag = true;
	_connected.store(false);

	if(_updater != nullptr)
	{
		try
		{
			_updater->join();
		}
		catch(boost::thread_interrupted &)
		{}

		delete _updater;
		_updater = nullptr;
	}
}

void Drone::runUpdateLoop()
{
	int runTime = 0;
	int updateInterval = 1000/40;

	boost::timer::cpu_timer timer;

	while(!_stop_flag)
	{
		// TODO: Think this through really well, find memory leaks!

		timer.start();

		// Check connection status and handle unexpected loss of connection
		if(!_connected.load()) {
			notifyConnectionLost();
			connectionLost();
			return;
		}

		// Call any miscellaneous functionality needed by implementation
		beforeUpdate();

		// Process command queue
		if(_commandqueue.empty())
		{
			_connected.store(processNoCommand());
			if(!_connected.load())
			{
				continue;
			}
		}
		else
		{
			for(drone::command command : _commandqueue)
			{
				_connected.store(processCommand(command));
				if(!_connected.load())
				{
					_commandqueue.clear();
					continue;
				}
			}
		}
		_commandqueue.clear();

		// Retrieve and process navdata
		bool newNavdata = decodeNavdata(_navdata);
		if(newNavdata)
		{
			notifyNavdataListeners(_navdata);
		}

		// Call any miscellaneous functionality needed by implementation
		updateCycle();

		// Run at continuous update rate
		runTime = timer.elapsed().wall / 1000000;
		timer.stop();

		if(updateInterval - runTime > 0)
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(updateInterval - runTime));
		}
	}
}

std::shared_ptr<drone::navdata> Drone::getNavdata()
{
	return _navdata;
}

bool Drone::addCommand(drone::command command)
{
	if(!_connected.load())
	{
		return false;
	}

	_commandqueue.push_back(command);

	return true;
}
