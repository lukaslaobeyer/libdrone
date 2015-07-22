#include "fullnavdata.h"

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

	memcpy(&roll, _navdata_buf.data() + 11 * sizeof(double), sizeof(double));
	memcpy(&pitch, _navdata_buf.data() + 12 * sizeof(double), sizeof(double));
	memcpy(&yaw, _navdata_buf.data() + 13 * sizeof(double), sizeof(double));
	memcpy(&height, _navdata_buf.data() + 28 * sizeof(double), sizeof(double));
	memcpy(&height_ultrasonic, _navdata_buf.data() + 26 * sizeof(double), sizeof(double));
	memcpy(&pressure, _navdata_buf.data() + 27 * sizeof(double), sizeof(double));
	memcpy(&vbat, _navdata_buf.data() + 69 * sizeof(double), sizeof(double));
	memcpy(&latitude, _navdata_buf.data() + 92 * sizeof(double), sizeof(double));
	memcpy(&longitude, _navdata_buf.data() + 93 * sizeof(double), sizeof(double));
	memcpy(&gps_altitude, _navdata_buf.data() + 94 * sizeof(double), sizeof(double));
	memcpy(&bat_percent, _navdata_buf.data() + 132 * sizeof(double), sizeof(double));
	memcpy(&speed_x, _navdata_buf.data() + 23 * sizeof(double), sizeof(double));
	memcpy(&speed_y, _navdata_buf.data() + 24 * sizeof(double), sizeof(double));
	memcpy(&speed_z, _navdata_buf.data() + 45 * sizeof(double), sizeof(double));
	memcpy(&gps_accuracy, _navdata_buf.data() + 97 * sizeof(double), sizeof(double));

	_navdata.altitude = height;
	_navdata.attitude = Eigen::Vector3f(pitch, roll, yaw);
	_navdata.batterystatus = bat_percent / 100.0f;
	_navdata.full = true;
	_navdata.gps_altitude = gps_altitude;
	_navdata.latitude = latitude;
	_navdata.linearvelocity = Eigen::Vector3f(speed_x, speed_y, speed_z);
	_navdata.longitude = longitude;

	_navdata.full_navdata.ultrasound_height = height_ultrasonic;
	_navdata.full_navdata.pressure = pressure;
	_navdata.full_navdata.vbat = vbat;
	_navdata.full_navdata.gps_accuracy = gps_accuracy;

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
