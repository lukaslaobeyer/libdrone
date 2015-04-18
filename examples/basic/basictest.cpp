#include <drones/ardrone2/ardrone2.h>
#include <drones/bebop/bebop.h>

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

using namespace std;

enum drone_type
{
	BEBOP,
	ARDRONE2
};

drone_type drone_type;
unique_ptr<Drone> _drone = nullptr;

class NavdataListener : public INavdataListener
{
	public:
	void navdataAvailable(std::shared_ptr<const drone::navdata> navdata)
	{
		cout << "Altitude: " << static_pointer_cast<const ardrone2::navdata>(navdata)->altitude << "m" << endl;
	}
};

int main(int argc, char **argv)
{
	// Check arguments and decide which drone to use (Bebop or AR.Drone 2.0)
	if(argc < 2)
	{
		cout << "Usage: " << argv[0] << " [Bebop|ARDrone2]" << endl;
		return -1;
	}
	if(strcmp(argv[1], "Bebop") == 0)
	{
		drone_type = BEBOP;
	}
	else if(strcmp(argv[1], "ARDrone2") == 0)
	{
		drone_type = ARDRONE2;
	}
	else
	{
		cout << "Drone " << argv[1] << " is unknown to this program." << endl;
		cout << "Supported drones: Bebop, ARDrone2" << endl;
		return -1;
	}

	cout << "[INFO]  Using drone " << argv[1] << "." << endl;

	// Setup drone and connect
	if(drone_type == ARDRONE2)
	{
		_drone.reset(new ARDrone2());
	}
	else
	{
		_drone.reset(new Bebop());
//		_drone->connect();
//		this_thread::sleep_for(chrono::milliseconds(1000));
//		return 1;
	}

	NavdataListener listener;
	_drone->addNavdataListener(&listener);

	drone::connectionstatus status = _drone->connect();
	if(status != drone::connectionstatus::CONNECTION_ESTABLISHED)
	{
		cout << "[ERROR] Could not connect to drone!" << endl;
		return 1;
	}
	else
	{
		cout << "[INFO]  Connection established!" << endl;
	}

	cout << "[INFO]  Update loop started" << endl;
	cout << "----------------------------------" << endl;

	// Do stuff!
	_drone->startUpdateLoop();

	this_thread::sleep_for(chrono::milliseconds(20000));

	_drone->stopUpdateLoop();

	cout << "----------------------------------" << endl;
	cout << "[INFO]  Update loop stopped" << endl;

	return 0;
}
