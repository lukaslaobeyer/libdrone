#include "fullnavdata.h"

#include <vector>
#include <utility>
#include <boost/log/trivial.hpp>
#include <boost/bind.hpp>

using namespace bebop;

using namespace std;
using boost::asio::ip::udp;

fullnavdata::fullnavdata()
{

}

void fullnavdata::init(std::string ip, boost::asio::io_service &io_service)
{
	_navdata_socket.reset(new udp::socket(io_service, udp::v4()));
	_navdata_socket->bind(udp::endpoint(udp::v4(), bebop::FULL_NAVDATA_PORT));

	udp::resolver resolver(io_service);
	udp::resolver::query query(ip, to_string(bebop::FULL_NAVDATA_PORT));
	endpoint = *resolver.resolve(query);

	_navdata_socket->send_to(boost::asio::buffer("hello"), endpoint);

	// Start receiving beautiful, full, informative (and not dumbed-down by Parrot) navdata ;)
	_navdata_socket->async_receive_from(boost::asio::buffer(_navdata_buf), _navdata_sender_endpoint, boost::bind(&fullnavdata::navdataPacketReceived, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void fullnavdata::navdataPacketReceived(const boost::system::error_code &error, size_t bytes_transferred)
{
	if(bytes_transferred < 1000)
	{
		return;
	}

	_gotNavdata = true;

	double pitch;
	double roll;
	double yaw;
	double height;
	double height_ultrasonic;
	double pressure;
	double vbat;
	double latitude;
	double longitude;
	double gps_sats;
	double gps_altitude;
	double bat_percent;
	double speed_x;
	double speed_y;
	double speed_z;
	double gps_accuracy;
	double gps_num_svs;
	double gps_eph, gps_epv;

	/*std::vector<std::pair<double *, int>> data_table = { // Firmware 3.2
			{&roll, 61},
			{&pitch, 62},
			{&yaw, 63},
			{&height, 76},
			{&height_ultrasonic, 15},
			{&pressure, 9},
			{&vbat, 58},
			{&latitude, 22},
			{&longitude, 23},
			{&gps_altitude, 24},
			{&bat_percent, 59},
			{&speed_x, 73},
			{&speed_y, 74},
			{&speed_z, 75},
			{&gps_accuracy, 27},
			{&gps_num_svs, 28},
			{&gps_eph, 32},
			{&gps_epv, 34}
	};*/

	std::vector<std::pair<double *, int>> data_table = { // Firmware 3.3
			{&roll, 100}, //ok
			{&pitch, 162}, //ok
			{&yaw, 110}, //ok
			{&height, 76}, //ok
			{&height_ultrasonic, 142}, //ok
			{&pressure, 141}, //ok
			{&vbat, 26}, //ok
			{&latitude, 126}, //ok
			{&longitude, 127}, //ok
			{&gps_altitude, 124}, //ok
			{&bat_percent, 25}, //ok
			{&speed_x, 148}, //ok
			{&speed_y, 149}, //ok
			{&speed_z, 150}, //ok
			{&gps_accuracy, 27},
			{&gps_num_svs, 128}, //ok
			{&gps_eph, 32},
			{&gps_epv, 34}
	};

	/*memcpy(&roll, _navdata_buf.data() + 11 * sizeof(double), sizeof(double)); // 62
	memcpy(&pitch, _navdata_buf.data() + 12 * sizeof(double), sizeof(double)); // 63
	memcpy(&yaw, _navdata_buf.data() + 13 * sizeof(double), sizeof(double)); // 64
	memcpy(&height, _navdata_buf.data() + 28 * sizeof(double), sizeof(double)); // 77
	memcpy(&height_ultrasonic, _navdata_buf.data() + 26 * sizeof(double), sizeof(double)); // 16
	memcpy(&pressure, _navdata_buf.data() + 27 * sizeof(double), sizeof(double)); // 10
	memcpy(&vbat, _navdata_buf.data() + 69 * sizeof(double), sizeof(double)); // 58 'raw'; 59 'compensated'
	memcpy(&latitude, _navdata_buf.data() + 92 * sizeof(double), sizeof(double)); // 23
	memcpy(&longitude, _navdata_buf.data() + 93 * sizeof(double), sizeof(double)); // 24
	memcpy(&gps_altitude, _navdata_buf.data() + 94 * sizeof(double), sizeof(double)); // 25
	memcpy(&bat_percent, _navdata_buf.data() + 132 * sizeof(double), sizeof(double)); // 60
	memcpy(&speed_x, _navdata_buf.data() + 23 * sizeof(double), sizeof(double)); // 74
	memcpy(&speed_y, _navdata_buf.data() + 24 * sizeof(double), sizeof(double)); // 75
	memcpy(&speed_z, _navdata_buf.data() + 45 * sizeof(double), sizeof(double)); // 76
	memcpy(&gps_accuracy, _navdata_buf.data() + 97 * sizeof(double), sizeof(double)); // 28
	memcpy(&gps_num_svs, _navdata_buf.data() + 98 * sizeof(double), sizeof(double)); // 29
	memcpy(&gps_eph, _navdata_buf.data() + 112 * sizeof(double), sizeof(double)); // 33; 34
	memcpy(&gps_epv, _navdata_buf.data() + 113 * sizeof(double), sizeof(double)); // 35*/
	for(const std::pair<double *, int> &element : data_table)
	{
		memcpy(element.first, _navdata_buf.data() + element.second * sizeof(double), sizeof(double));
	}

	_navdata.altitude = height;
	_navdata.attitude = Eigen::Vector3f(pitch, roll, yaw);
	_navdata.batterystatus = bat_percent / 100.0f;
	_navdata.full = true;
	_navdata.gps_altitude = gps_altitude;
	_navdata.gps_sats = gps_num_svs;
	_navdata.latitude = latitude;
	_navdata.linearvelocity = Eigen::Vector3f(speed_x, speed_y, speed_z);
	_navdata.longitude = longitude;

	_navdata.full_navdata.ultrasound_height = height_ultrasonic;
	_navdata.full_navdata.pressure = pressure;
	_navdata.full_navdata.vbat = vbat;
	_navdata.full_navdata.gps_accuracy = gps_accuracy;
	_navdata.full_navdata.gps_eph = gps_eph;
	_navdata.full_navdata.gps_epv = gps_epv;

	// Listen for next packet
	_navdata_socket->async_receive_from(boost::asio::buffer(_navdata_buf), _navdata_sender_endpoint, boost::bind(&fullnavdata::navdataPacketReceived, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

shared_ptr<navdata> fullnavdata::getNavdata()
{
	if(!_gotNavdata)
	{
		return nullptr;
	}

	shared_ptr<navdata> navdata = make_shared<bebop::navdata>();

	*navdata = _navdata; // Copy local navdata

	return navdata;
}
