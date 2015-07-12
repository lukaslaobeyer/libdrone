#include "controllink.h"

#include <boost/log/trivial.hpp>

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <thread>
#include <chrono>
#include <locale>

using namespace bebop;

using namespace std;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;

controllink::controllink()
{
	_navdata.altitude = 0;
	_navdata.attitude = Eigen::Vector3f();
	_navdata.batterystatus = 0;
	_navdata.cameraorientation = Eigen::Vector2f();
	_navdata.flying = false;
	_navdata.gps_altitude = 0;
	_navdata.latitude = 0;
	_navdata.linearvelocity = Eigen::Vector3f();
	_navdata.linkquality = 0;
	_navdata.longitude = 0;
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
	int videoFragmentSize = discovery_response.get<int>("arstream_fragment_size");
	int videoMaxFragmentNumber = discovery_response.get<int>("arstream_fragment_maximum_number");

	BOOST_LOG_TRIVIAL(debug) << "fragment size:   " << videoFragmentSize;
	BOOST_LOG_TRIVIAL(debug) << "fragment number: " << videoMaxFragmentNumber;

	if(videoFragmentSize > BEBOP_NAVDATA_BUFFER_SIZE)
	{
		BOOST_LOG_TRIVIAL(fatal) << "Framebuffer not big enough! Video fragment size > allocated memory!";
	}

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

	/////// INITIALIZE VIDEO DECODER ///////
	_videodecoder.reset(new videodecoder(videoFragmentSize, videoMaxFragmentNumber));
}

void controllink::initConfig()
{
	// Enable video
	navdata_id videoen_id = command_ids::enable_streaming;
	vector<boost::any> vid_args = {(uint8_t) 1};
	sendAckedCommand(videoen_id, vid_args, navdata_ids::streaming_state);

	// Flat trim
	navdata_id flattrim_id = command_ids::flattrim;
	vector<boost::any> ft_args = {};
	sendAckedCommand(flattrim_id, ft_args, navdata_ids::flattrim);

	// Set time and date
	navdata_id date_id = command_ids::setdate;
	string date = boost::gregorian::to_iso_extended_string(boost::gregorian::day_clock::universal_day());
	BOOST_LOG_TRIVIAL(debug) << date;
	vector<boost::any> date_args = {date};
	sendAckedCommand(date_id, date_args, navdata_ids::date);

	navdata_id time_id = command_ids::settime;
	boost::posix_time::ptime time = boost::posix_time::second_clock::universal_time();
	boost::posix_time::time_facet *facet = new boost::posix_time::time_facet;
	facet->format("T%H%M%S+0000");
	stringstream timestream;
	timestream.imbue(locale(locale::classic(), facet));
	timestream << time;
	string time_str = timestream.str();
	BOOST_LOG_TRIVIAL(debug) << time_str;
	vector<boost::any> time_args = {time_str};
	sendAckedCommand(time_id, time_args, navdata_ids::time);

	// Get Bebop status
	navdata_id getstatus_id = command_ids::getstatus;
	vector<boost::any> gs_args = {};
	sendAckedCommand(getstatus_id, gs_args, navdata_ids::all_states_sent);

	// 720P?
	/*navdata_id video720p_id = command_ids::stream_720p;
	vector<boost::any> vid720p_args = {(uint64_t) 1 << 0};
	sendCommand(video720p_id, vid720p_args);*/

	/*navdata_id autorecord_id = command_ids::video_autorecord;
	vector<boost::any> ar_args = {(uint8_t) 0, (uint8_t) 0};
	sendCommand(autorecord_id, ar_args);*/
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

		// Find out type of data (see ARNETWORKAL_Frame.h in libARNetworkAL of the official SDK)
		switch(_navdata_receivedDataBuffer[0])
		{
		case frametype::DATA:
			// Regular data

			switch(_navdata_receivedDataBuffer[1])
			{
			case frameid::PING:
				// Request for pong packet
				sendPong(_navdata_receivedDataBuffer, bytes_transferred);
				break;
			case frameid::PONG:
				// Confirmation that pong packet was received
				break;
			case frameid::NAVDATA:
				// Navigation data packet
				decodeNavdataPacket(_navdata_receivedDataBuffer, bytes_transferred);
				break;
			default:
				BOOST_LOG_TRIVIAL(debug) << "UNKNOWN NAVDATA ID: 0x" << hex << (int) _navdata_receivedDataBuffer[1] << dec;
				break;
			}

			break;
		case frametype::LOW_LATENCY:
			// Low latency data
			switch(_navdata_receivedDataBuffer[1])
			{
			case frameid::VIDEO_WITH_ACK:
				sendVideoAck(_navdata_receivedDataBuffer, bytes_transferred);
				decodeVideoPacket(_navdata_receivedDataBuffer, bytes_transferred);
				break;
			default:
				BOOST_LOG_TRIVIAL(debug) << "UNKNOWN NAVDATA ID: 0x" << hex << (int) _navdata_receivedDataBuffer[1] << dec;
			}
			break;
		case frametype::DATA_WITH_ACK:
			// Data with ack
			switch(_navdata_receivedDataBuffer[1])
			{
			case frameid::NAVDATA_WITH_ACK:
				decodeNavdataPacket(_navdata_receivedDataBuffer, bytes_transferred);
				sendAck(_navdata_receivedDataBuffer, bytes_transferred);
				break;
			default:
				BOOST_LOG_TRIVIAL(debug) << "UNKNOWN NAVDATA ID: 0x" << hex << (int) _navdata_receivedDataBuffer[1] << dec;
			}
			break;
		default:
			BOOST_LOG_TRIVIAL(debug) << "UNKNOWN FRAMETYPE: 0x" << hex << (int) _navdata_receivedDataBuffer[0] << dec;
			break;
		}
	}
	else
	{
		//cout << "ERR " << error << endl;
	}

	// Receive next packet
	startReceivingNavdata();
}

void controllink::sendPong(d2cbuffer &receivedDataBuffer, size_t bytes_received)
{
	// Send a "pong" packet as reply to a ping packet. Contents are a copy of the ping packet.
	_c2d_socket->send(boost::asio::buffer(receivedDataBuffer, bytes_received));
}

void controllink::sendAck(d2cbuffer &receivedDataBuffer, size_t bytes_received)
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

void controllink::sendVideoAck(d2cbuffer &receivedDataBuffer, size_t bytes_received)
{
	// Send an acknowledge message to signal that a video fragment was received.

	static uint8_t seqNum = 0;

	static uint16_t lastFrameIndex = 0;
	static uint64_t highPacketsAck = 0, lowPacketsAck = 0;

	// Calculate ack packet payload contents
	uint16_t frameIndex;
	memcpy(&frameIndex, &receivedDataBuffer[7], sizeof(uint16_t));
	uint8_t frameFlags = receivedDataBuffer[9];
	uint8_t fragmentIndex = receivedDataBuffer[10];
	uint8_t fragmentsInFrame = receivedDataBuffer[11];

	if(frameIndex != lastFrameIndex) // The fragment to be acked is not part of the frame the previous fragment belonged to
	{
		highPacketsAck = 0;
		lowPacketsAck = 0;
		lastFrameIndex = frameIndex;
	}

	if(fragmentsInFrame < 64) // Calculate ack bitmasks
	{
		lowPacketsAck |= ((uint64_t) 1 << (uint64_t) fragmentIndex);
	}
	else
	{
		highPacketsAck |= ((uint64_t) 1 << ((uint64_t) fragmentIndex - (uint64_t) 64));
	}

	//BOOST_LOG_TRIVIAL(trace) << bitset<64>(highPacketsAck) << "; " << bitset<64>(lowPacketsAck);

	// Prepare the packet for sending
	frameheader header;
	header.type = frametype::DATA;
	header.id = frameid::VIDEO_ACK_RESPONSE;
	header.seq = seqNum;

	vector<char> payload(sizeof(uint16_t) + 2*sizeof(uint64_t));
	memcpy(&payload.data()[0], &frameIndex, sizeof(uint16_t));
	memcpy(&payload.data()[2], &highPacketsAck, sizeof(uint64_t));
	memcpy(&payload.data()[10], &lowPacketsAck, sizeof(uint64_t));

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

void controllink::sendAckedCommand(navdata_id command_id, vector<boost::any> args, navdata_id ack_id)
{
	_command_id_queue.push(command_id);
	_command_arg_queue.push(args);
	_ack_id_queue.push(ack_id);
}

void controllink::processCommandQueue()
{
	if(_command_id_queue.size() > 0 && !_ack_expected)
	{
		sendCommand(_command_id_queue.front(), _command_arg_queue.front());
		_ack_expected = true;
	}
}

shared_ptr<navdata> controllink::getNavdata()
{
	shared_ptr<navdata> navdata = make_shared<bebop::navdata>();

	*navdata = _navdata; // Copy local navdata

	return navdata;
}

cv::Mat controllink::getVideoFrame()
{
	return _videodecoder->getLatestFrame();
}

void controllink::decodeNavdataPacket(d2cbuffer &receivedDataBuffer, size_t bytes_transferred)
{
	static int ackWaitCycles;

	uint8_t  dataDevice = receivedDataBuffer[7]; // See the XML files in libARCommands of the official SDK
	uint8_t  dataClass  = receivedDataBuffer[8];
	uint16_t dataID     = receivedDataBuffer[10] << 8 | receivedDataBuffer[9];

	navdata_id navdata_key{dataDevice, dataClass, dataID};

	// Check for ack data if expected
	if(_ack_expected)
	{
		if(navdata_key == _ack_id_queue.front())
		{
			// Expected ack packet received
			_ack_id_queue.pop();
			_command_arg_queue.pop();
			_command_id_queue.pop();
			_ack_expected = false;

			ackWaitCycles = 0;

			BOOST_LOG_TRIVIAL(debug) << "Ack received";
		}
		else
		{
			ackWaitCycles++;

			if(ackWaitCycles >= MAX_ACK_WAIT_CYCLES)
			{
				BOOST_LOG_TRIVIAL(warning) << "No ack received! Packet ignored";

				// Skip packet
				_ack_id_queue.pop();
				_command_arg_queue.pop();
				_command_id_queue.pop();
				_ack_expected = false;

				ackWaitCycles = 0;
			}
		}
	}

	// Decode navdata
	if(navdata_key == navdata_ids::wifi_rssi)
	{
		int16_t wifi_rssi; // in dBm
		memcpy(&wifi_rssi, &_navdata_receivedDataBuffer.data()[11], sizeof(wifi_rssi));

		int link_quality = min(2*(wifi_rssi+100), 100); // in %, according to Microsoft

		_navdata.linkquality = (float) link_quality / 100.0f;
	}
	else if(navdata_key == navdata_ids::battery_status)
	{
		uint8_t battery_percentage = _navdata_receivedDataBuffer[11];

		_navdata.batterystatus = (float) battery_percentage / 100.0f;
	}
	else if(navdata_key == navdata_ids::altitude)
	{
		double altitude;
		memcpy(&altitude, &_navdata_receivedDataBuffer.data()[11], sizeof(double));	memcpy(&altitude, &_navdata_receivedDataBuffer.data()[11], sizeof(double));

		_navdata.altitude = altitude;
	}
	else if(navdata_key == navdata_ids::attitude)
	{
		float roll, pitch, yaw;
		memcpy(&roll, &_navdata_receivedDataBuffer.data()[11], sizeof(float));
		memcpy(&pitch, &_navdata_receivedDataBuffer.data()[11 + 4], sizeof(float));
		memcpy(&yaw, &_navdata_receivedDataBuffer.data()[11 + 4*2], sizeof(float));

		_navdata.attitude = Eigen::Vector3f(pitch, roll, yaw);
	}
	else if(navdata_key == navdata_ids::speed)
	{
		float speedX, speedY, speedZ;
		memcpy(&speedX, &_navdata_receivedDataBuffer.data()[11], sizeof(float));
		memcpy(&speedY, &_navdata_receivedDataBuffer.data()[11 + 4], sizeof(float));
		memcpy(&speedZ, &_navdata_receivedDataBuffer.data()[11 + 4*2], sizeof(float));

		_navdata.linearvelocity = Eigen::Vector3f(speedX, speedY, speedZ);
	}
	else if(navdata_key == navdata_ids::gps)
	{
		double lat, lon, alt;
		memcpy(&lat, &_navdata_receivedDataBuffer.data()[11], sizeof(double));
		memcpy(&lon, &_navdata_receivedDataBuffer.data()[11 + 8], sizeof(double));
		memcpy(&alt, &_navdata_receivedDataBuffer.data()[11 + 8*2], sizeof(double));

		_navdata.latitude = lat;
		_navdata.longitude = lon;
		_navdata.gps_altitude = alt;
	}
	else if(navdata_key == navdata_ids::camera_orientation)
	{
		int8_t tilt = _navdata_receivedDataBuffer[11];
		int8_t pan = _navdata_receivedDataBuffer[12];

		_navdata.cameraorientation = Eigen::Vector2f(tilt, pan);
	}
	else if(navdata_key == navdata_ids::flattrim)
	{
		BOOST_LOG_TRIVIAL(debug) << "flat trim ack'd";
	}
	else if(navdata_key == navdata_ids::streaming_state)
	{
		uint8_t state = _navdata_receivedDataBuffer[11];

		BOOST_LOG_TRIVIAL(debug) << "streaming state changed: " << (int) state;
	}
	else if(navdata_key == navdata_ids::flying_state)
	{
		uint8_t state = _navdata_receivedDataBuffer[11];

		if(state == 0 || state == 5)
		{
			_navdata.flying = false;
		}
		else
		{
			_navdata.flying = true;
		}
	}
	else if(navdata_key == navdata_ids::alert_state)
	{
		uint8_t state = _navdata_receivedDataBuffer[11];

		BOOST_LOG_TRIVIAL(debug) << "alert state changed: " << (int) state;
	}
	else if(navdata_key == navdata_ids::picture_taken)
	{
		uint8_t state = _navdata_receivedDataBuffer[11];

		BOOST_LOG_TRIVIAL(debug) << "picture taken? " << (int) state;
	}
	else if(navdata_key == navdata_ids::video_recording_state)
	{
		uint8_t state = _navdata_receivedDataBuffer[11];

		BOOST_LOG_TRIVIAL(debug) << "video recording? " << (int) state;
	}
	else
	{
		BOOST_LOG_TRIVIAL(debug) << "UNKNOWN NAVDATA: " << (int) dataDevice << "; " << (int) dataClass << "; " << (int) dataID;
	}
}

bool controllink::decodeVideoPacket(d2cbuffer &receivedDataBuffer, size_t bytes_transferred)
{
	uint16_t frameIndex;
	memcpy(&frameIndex, &receivedDataBuffer[7], sizeof(uint16_t));
	uint8_t frameFlags = receivedDataBuffer[9];
	uint8_t fragmentIndex = receivedDataBuffer[10];
	uint8_t fragmentsInFrame = receivedDataBuffer[11];
	
    int frame_size = bytes_transferred - 12;
    
    //BOOST_LOG_TRIVIAL(debug) << "n = " << frameIndex << "; " << (int) fragmentIndex+1 << "/" << (int) fragmentsInFrame << "; " << (int) bytes_transferred;

	bool fragmentValid = _videodecoder->insertFragment(receivedDataBuffer, frameIndex, fragmentsInFrame, fragmentIndex, frame_size);
	
	return fragmentValid;
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
		else if(command_id.dataTypes[i] == 'i') // int32_t
		{
			int32_t arg = boost::any_cast<int32_t>(args[i]);
			char arg8[4];
			memcpy(&arg8, &arg, sizeof(int32_t));
			command.insert(command.end(), {arg8[0], arg8[1], arg8[2], arg8[3]});
		}
		else if(command_id.dataTypes[i] == 'I') // uint32_t
		{
			uint32_t arg = boost::any_cast<uint32_t>(args[i]);
			char arg8[4];
			memcpy(&arg8, &arg, sizeof(uint32_t));
			command.insert(command.end(), {arg8[0], arg8[1], arg8[2], arg8[3]});
		}
		else if(command_id.dataTypes[i] == 'l') // int64_t
		{
			int64_t arg = boost::any_cast<int64_t>(args[i]);
			char arg8[8];
			memcpy(&arg8, &arg, sizeof(int64_t));
			command.insert(command.end(), {arg8[0], arg8[1], arg8[2], arg8[3], arg8[4], arg8[5], arg8[6], arg8[7]});
		}
		else if(command_id.dataTypes[i] == 'L') // uint64_t
		{
			uint64_t arg = boost::any_cast<uint64_t>(args[i]);
			char arg8[8];
			memcpy(&arg8, &arg, sizeof(uint64_t));
			command.insert(command.end(), {arg8[0], arg8[1], arg8[2], arg8[3], arg8[4], arg8[5], arg8[6], arg8[7]});
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
		else if(command_id.dataTypes[i] == 's')
		{
			string arg = boost::any_cast<string>(args[i]);
			copy(arg.begin(), arg.end(), back_inserter(command));
			command.insert(command.end(), '\0');
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
