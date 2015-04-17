#include "controllink.h"

#include <array>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace bebop;

using namespace std;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;

controllink::controllink()
{

}

void controllink::init(string ip, boost::asio::io_service &io_service)
{
	/////// INITIALIZE BEBOP CONNECTION ///////
	
	// Bebop discovery connection
	tcp::resolver discovery_resolver(io_service);
	tcp::resolver::query discovery_query(ip, to_string(bebop::DISCOVERY_PORT));
	tcp::endpoint discovery_endpoint = *resolver.resolve(discovery_query);

	tcp::socket discovery_socket(io_service);
	discovery_socket.open(tcp::v4());
	discovery_socket.bind(tcp::endpoint(tcp::v4(), bebop::DISCOVERY_PORT));
	discovery_socket.connect(discovery_endpoint);

	// Bebop configuration packet
	boost::property_tree::ptree discovery_request;
	discovery_request.put("controller_type", bebop::discovery::controller_type);
	discovery_request.put("controller_name", bebop::discovery::controller_name);
	discovery_request.put("d2c_port", bebop::D2C_PORT);

	// As JSON
	ostringstream discovery_request_buf;
	boost::property_tree::write_json(discovery_request_buf, discovery_request, false);

	discovery_socket.send(boost::asio::buffer(discovery_request_buf.str()));

	// Read response (should contain c2d_port ["controller-to-drone"] and other misc. information
	stringstream discovery_response_buf;
	for(;;)
	{
		array<char, 512> buf;
		boost::system::error_code error;

		size_t len = discovery_socket.read_some(boost::asio::buffer(buf), error);

		if(error == boost::asio::error::eof)
		{
			break;
		}
		else if(error)
		{
			throw boost::system::system_error(error);
		}

		discovery_response_buf << string(buf.data(), len).c_str(); // Weirdness required so that boost property tree does not complain about end of input
	}

	// Parse response (as JSON)
	boost::property_tree::ptree discovery_response;
	boost::property_tree::read_json(discovery_response_buf, discovery_response);

	int c2d_port = discovery_response.get<int>("c2d_port");
	cout << "C2D port: " << c2d_port << endl;
	
	/////// INITIALIZE NAVDATA CONNECTION ///////
	udp::resolver navdata_resolver(io_service);
	udp::resolver::query navdata_query(udp::v4(), ip, to_string(c2d_port));
	udp::endpoint navdata_endpoint = *resolver.resolve(navdata_query);
	
	_navdata_socket = unique_ptr(new udp::socket(io_service)); //TODO: C++14: Use make_unique
	_navdata_socket->open(udp::v4());
	_navdata_socket->bind(udp::endpoint(udp::v4(), c2d_port));
	_navdata_socket->connect(navdata_endpoint);
}

void controllink::startReceive()
{
    _navdata_socket->async_receive_from(boost::asio::buffer(_receivedDataBuffer), sender_endpoint, boost::bind(&NavdataManager::packetReceived, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}