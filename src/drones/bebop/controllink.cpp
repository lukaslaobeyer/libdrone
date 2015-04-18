#include "controllink.h"

#include "protocol.h"

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
	
	/////// INITIALIZE NAVDATA CONNECTION ///////
	_d2c_socket.reset(new udp::socket(io_service, udp::v4()));
	_d2c_socket->bind(udp::endpoint(udp::v4(), bebop::D2C_PORT));

	/////// INITIALIZE COMMAND CONNECTION ///////
	udp::resolver c2d_resolver(io_service);
	udp::resolver::query c2d_query(ip, to_string(c2d_port));
	udp::endpoint c2d_endpoint = *c2d_resolver.resolve(c2d_query);

	_c2d_socket.reset(new udp::socket(io_service, udp::v4()));
	_c2d_socket->bind(udp::endpoint(udp::v4(), c2d_port));
	_c2d_socket->connect(c2d_endpoint);

	// Start receiving packets
	startReceivingNavdata();
}

void controllink::startReceivingNavdata()
{
	_d2c_socket->async_receive_from(boost::asio::buffer(_navdata_receivedDataBuffer), _navdata_sender_endpoint, boost::bind(&controllink::navdataPacketReceived, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void controllink::navdataPacketReceived(const boost::system::error_code &error, size_t bytes_transferred)
{
	if(!error)
	{
		uint8_t seqNum = _navdata_receivedDataBuffer[2];

		uint32_t size = (uint32_t)_navdata_receivedDataBuffer[6] << 24 |
					    (uint32_t)_navdata_receivedDataBuffer[5] << 16 |
					    (uint32_t)_navdata_receivedDataBuffer[4] << 8  |
					    (uint32_t)_navdata_receivedDataBuffer[3];

		cout << (int) seqNum << "\t" << size << "\t";

		// Find out type of data (see ARNETWORKAL_Frame.h in libARNetworkAL of the official SDK)
		switch(_navdata_receivedDataBuffer[0])
		{
		case frametype::ACK:
			// Ack packet
			cout << "ack";
			break;
		case frametype::DATA:
			// Regular data
			cout << "data";

			switch(_navdata_receivedDataBuffer[1])
			{
			case frameid::PING:
				// Request for pong packet
				cout << " ping";
				sendPong(_navdata_receivedDataBuffer, bytes_transferred);
				break;
			case frameid::PONG:
				// Confirmation that pong packet was received
				cout << " pong";
				break;
			case frameid::NAVDATA:
				// Navigation data packet
				cout << " navdata";
				decodeNavdataPacket(_navdata_receivedDataBuffer, bytes_transferred);
				break;
			}

			break;
		case frametype::LOW_LATENCY:
			// Low latency data
			cout << "low latency data";
			break;
		case frametype::DATA_WITH_ACK:
			// Data with ack
			cout << "data with ack";
			break;
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

void controllink::sendPong(d2cbuffer receivedDataBuffer, size_t bytes_received)
{
	_c2d_socket->send(boost::asio::buffer(receivedDataBuffer, bytes_received));
}

void controllink::sendCommand(vector<char> &command)
{
	vector<char> packet;
	packet.reserve(_command_header.size() + command.size());
	packet.insert(packet.end(), _command_header.begin(), _command_header.end());
	packet.insert(packet.end(), command.begin(), command.end());

	_c2d_socket->send(boost::asio::buffer(packet));
}

void controllink::decodeNavdataPacket(d2cbuffer receivedDataBuffer, size_t bytes_transferred)
{
	uint8_t  dataDevice = receivedDataBuffer[7]; // See the XML files in libARCommands of the official SDK
	uint8_t  dataClass  = receivedDataBuffer[8];
	uint16_t dataID     = receivedDataBuffer[10] << 8 | receivedDataBuffer[9];

	navdata_id navdata_key{dataDevice, dataClass, dataID};

	if(navdata_key == navdata_ids::wifi_rssi)
	{
		cout << "\twifi\t";
	}
	else if(navdata_key == navdata_ids::altitude)
	{
		cout << "\taltitude\t\t";
		double altitude;
		memcpy(&altitude, &_navdata_receivedDataBuffer.data()[11], sizeof(double));
		cout << altitude;
	}
	else if(navdata_key == navdata_ids::attitude)
	{
		cout << "\tattitude\t\t";
		float roll, pitch, yaw;
		memcpy(&roll, &_navdata_receivedDataBuffer.data()[11], sizeof(float));
		memcpy(&pitch, &_navdata_receivedDataBuffer.data()[11 + 4], sizeof(float));
		memcpy(&yaw, &_navdata_receivedDataBuffer.data()[11 + 4*2], sizeof(float));
		cout << roll * (360/(2*3.1416)) << "\t";
		cout << pitch * (360/(2*3.1416)) << "\t";
		cout << yaw * (360/(2*3.1416));
	}
	else if(navdata_key == navdata_ids::speed)
	{
		cout << "\tspeed\t\t\t";
		float speedX, speedY, speedZ;
		memcpy(&speedX, &_navdata_receivedDataBuffer.data()[11], sizeof(float));
		memcpy(&speedY, &_navdata_receivedDataBuffer.data()[11 + 4], sizeof(float));
		memcpy(&speedZ, &_navdata_receivedDataBuffer.data()[11 + 4*2], sizeof(float));
		cout << speedX << "\t";
		cout << speedY << "\t";
		cout << speedZ;
	}
	else if(navdata_key == navdata_ids::gps)
	{
		cout << "\tgps\t\t\t";
	}
	else if(navdata_key == navdata_ids::camera_orientation)
	{
		cout << "\tcamera orientation\t";
	}
}
