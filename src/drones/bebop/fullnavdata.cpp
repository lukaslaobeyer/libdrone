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

	memcpy(&roll, _navdata_buf.data() + 11 * sizeof(double), sizeof(double));
	memcpy(&pitch, _navdata_buf.data() + 12 * sizeof(double), sizeof(double));
	memcpy(&yaw, _navdata_buf.data() + 13 * sizeof(double), sizeof(double));

	_navdata.attitude = Eigen::Vector3f(pitch, roll, yaw);

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
