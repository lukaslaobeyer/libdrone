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
/*
	// Flat trim
	navdata_id flattrim_id = command_ids::flattrim;
	vector<boost::any> ft_args = {};
	sendCommand(flattrim_id, ft_args);

	// Enable video
	navdata_id videoen_id = command_ids::enable_streaming;
	vector<boost::any> vid_args = {(uint8_t) 1};
	sendCommand(videoen_id, vid_args);
*/
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

		uint32_t size;
		memcpy(&size, &_navdata_receivedDataBuffer[3], sizeof(uint32_t));

		cout << (int) seqNum << "\t" << size << "\t";

		// Find out type of data (see ARNETWORKAL_Frame.h in libARNetworkAL of the official SDK)
		switch(_navdata_receivedDataBuffer[0])
		{
		case frametype::ACK:
			// Ack packet
			cout << "  ack";
			break;
		case frametype::DATA:
			// Regular data
			cout << " data";

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
			cout << "ack'd";
			if(frameid::NAVDATA_WITH_ACK == _navdata_receivedDataBuffer[1])
			{
				cout << " navdata";
				decodeNavdataPacket(_navdata_receivedDataBuffer, bytes_transferred);
				sendAck(_navdata_receivedDataBuffer, bytes_transferred);
				cout << " [ack'd!]";
			}
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
	// Send a "pong" packet as reply to a ping packet. Contents are a copy of the ping packet.
	_c2d_socket->send(boost::asio::buffer(receivedDataBuffer, bytes_received));
}

void controllink::sendAck(d2cbuffer receivedDataBuffer, size_t bytes_received)
{
	// Send an acknowledge message to signal that packet was received. Only required for special packets with frame type frametype::DATA_WITH_ACK.

	static uint8_t seqNum = 0;

	frameheader header;
	header.type = frametype::ACK;
	header.id = frameid::ACK_RESPONSE;
	header.seq = seqNum;

	vector<char> payload = {_navdata_receivedDataBuffer[2]}; // Single byte as payload containing the sequence number of the received frame requesting ack
	vector<char> packet = assemblePacket(header, payload);

	_c2d_socket->send(boost::asio::buffer(packet));

	seqNum++;
}

void controllink::sendCommand(navdata_id &command_id, vector<boost::any> &args)
{
	// Send a command. The command_id is combined with the packet header to form a command header, then data is extracted from the
	// arguments passed with &args and sent to the c2d port.

	static uint8_t seqNum = 0;

	frameheader header;
	header.type = frametype::DATA;
	header.id = frameid::COMMAND;
	header.seq = seqNum;

	vector<char> command = createCommand(command_id, args);
	vector<char> packet = assemblePacket(header, command);

	_c2d_socket->send(boost::asio::buffer(packet));

	seqNum++;
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
	else if(navdata_key == navdata_ids::flattrim)
	{
		cout << "\tflat trim ack'd\t";
	}
	else if(navdata_key == navdata_ids::streaming_state)
	{
		cout << "\tstreaming state changed\t";
	}
	else
	{
		cout << "\tUNKNOWN\t";
	}
}

vector<char> controllink::createCommand(navdata_id &command_id, vector<boost::any> &args)
{
	char dataID[2];
	memcpy(dataID, &command_id.dataID, sizeof(uint16_t));
	vector<char> command = {(char) command_id.dataDevice, (char) command_id.dataClass, dataID[0], dataID[1]};

	if(command_id.dataTypes.size() != args.size())
	{
		throw InvalidCommandException();
	}

	for(int i = 0; i < command_id.dataTypes.size(); i++) // Add the arguments
	{
		if(command_id.dataTypes[i] == 'b') // int8_t
		{
			int8_t arg = boost::any_cast<int8_t>(args[i]);
			command.push_back(arg);
		}
		else if(command_id.dataTypes[i] == 'B') // uint8_t
		{
			uint8_t arg = boost::any_cast<uint8_t>(args[i]);
			command.push_back(arg);
		}
		else if(command_id.dataTypes[i] == 'h') // int16_t
		{
			int16_t arg = boost::any_cast<int16_t>(args[i]);
			char arg8[2];
			memcpy(&arg8, &arg, sizeof(int16_t));
			command.insert(command.end(), {arg8[0], arg8[1]});
		}
		else if(command_id.dataTypes[i] == 'H') // uint16_t
		{
			uint16_t arg = boost::any_cast<uint16_t>(args[i]);
			char arg8[2];
			memcpy(&arg8, &arg, sizeof(uint16_t));
			command.insert(command.end(), {arg8[0], arg8[1]});
		}
		else if(command_id.dataTypes[i] == 'f') // float
		{
			float arg = boost::any_cast<float>(args[i]);
			char arg8[4];
			memcpy(&arg8, &arg, sizeof(float));
			command.insert(command.end(), {arg8[0], arg8[1], arg8[2], arg8[3]});
		}
		else if(command_id.dataTypes[i] == 'd') // double
		{
			double arg = boost::any_cast<double>(args[i]);
			char arg8[8];
			memcpy(&arg8, &arg, sizeof(double));
			command.insert(command.end(), {arg8[0], arg8[1], arg8[2], arg8[3], arg8[4], arg8[5], arg8[6], arg8[7]});
		}
	}

	return command;
}

frameheaderbuf controllink::createHeader(frameheader &header)
{
	frameheaderbuf buf;

	buf[0] = header.type;
	buf[1] = header.id;
	buf[2] = header.seq;

	buf[3] = (uint8_t) header.size;
    buf[4] = (uint8_t)(header.size>>=8);
    buf[5] = (uint8_t)(header.size>>=8);
    buf[6] = (uint8_t)(header.size>>=8);

    return buf;
}

vector<char> controllink::assemblePacket(frameheader &header, vector<char> &payload)
{
	// Assemble a complete packet from the provided frameheader (size not required, will be calculated by this function) and payload

	frameheaderbuf frameHeaderBuf;

	uint32_t size = frameHeaderBuf.size() + payload.size(); // Calculate and set size
	header.size = size;

	frameHeaderBuf = createHeader(header);

	vector<char> packet;
	packet.reserve(size);
	packet.insert(packet.end(), frameHeaderBuf.begin(), frameHeaderBuf.end());
	packet.insert(packet.end(), payload.begin(), payload.end());

	return packet;
}
