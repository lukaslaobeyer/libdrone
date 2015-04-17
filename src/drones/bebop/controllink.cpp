#include "controllink.h"

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
	tcp::endpoint discovery_endpoint = *discovery_resolver.resolve(discovery_query);

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
	_navdata_socket.reset(new udp::socket(io_service, udp::v4()));
	_navdata_socket->bind(udp::endpoint(udp::v4(), bebop::D2C_PORT));

	// Start receiving packets
	startReceivingNavdata();
}

void controllink::startReceivingNavdata()
{
	_navdata_socket->async_receive_from(boost::asio::buffer(_navdata_receivedDataBuffer), _navdata_sender_endpoint, boost::bind(&controllink::navdataPacketReceived, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void controllink::navdataPacketReceived(const boost::system::error_code &error, std::size_t bytes_transferred)
{
	if(!error)
	{
		//cout << bytes_transferred << endl;
		cout << hex << (int) _navdata_receivedDataBuffer[0] << " " << (int) _navdata_receivedDataBuffer[1];
		cout << " " << (int) _navdata_receivedDataBuffer[2] << " " << (int) _navdata_receivedDataBuffer[3] << dec;

		if(_navdata_receivedDataBuffer[0] == 0x2)
		{
			if(_navdata_receivedDataBuffer[1] == 0x7f)
			{
				if(_navdata_receivedDataBuffer[3] == 0x13)
				{
					cout << "\tnavdata";
				}
				else if(_navdata_receivedDataBuffer[3] == 0x17)
				{
					cout << "\tvideo";
				}
			}
			else if(_navdata_receivedDataBuffer[1] == 0)
			{
				if(_navdata_receivedDataBuffer[3] == 0xF)
				{
					cout << "\tping";
				}
			}
		}

		cout << endl;
	}
	else
	{
		//cout << "ERR " << error << endl;
	}

	// Receive next packet
	startReceivingNavdata();
}
