#ifndef LIBDRONE_BEBOP_CONTROLLINK_H
#define LIBDRONE_BEBOP_CONTROLLINK_H

#define MAX_ACK_WAIT_CYCLES 40

#include <drones/bebop/constants.h>
#include <types.h>
#include <commands.h>

#include "protocol.h"
#include "videodecoder.h"

#include <memory>
#include <array>
#include <vector>
#include <queue>

#include <boost/any.hpp>

#include <boost/asio.hpp>

#include <opencv2/opencv.hpp>

class InvalidCommandException : public std::runtime_error
{
	public:
		InvalidCommandException() : std::runtime_error("Invalid command (arguments do not match command requirements)") { }
};

namespace bebop
{
	typedef std::array<char, 7> frameheaderbuf;

	class controllink
	{
		public:
			explicit controllink();
			
			void init(std::string ip, boost::asio::io_service &io_service);
			void initConfig(); // Requires a running update loop
			
			void sendCommand(navdata_id &command_id, std::vector<boost::any> &args);

			void processCommandQueue();

			std::shared_ptr<navdata> getNavdata();
			cv::Mat getVideoFrame();

			static std::vector<char> createCommand(navdata_id &command_id, std::vector<boost::any> &args);
			static frameheaderbuf createHeader(frameheader &header);
			static std::vector<char> assemblePacket(frameheader &header, std::vector<char> &payload);
		private:
			void sendCommand(navdata_id &command_id, std::vector<boost::any> &args, bool ack);
			void sendAckedCommand(navdata_id &command_id, std::vector<boost::any> &args);

			void startReceivingNavdata();
			void navdataPacketReceived(const boost::system::error_code &error, std::size_t bytes_transferred);

			void sendPong(d2cbuffer &receivedDataBuffer, std::size_t bytes_transferred);
			void sendAck(d2cbuffer &receivedDataBuffer, size_t bytes_received);
			void sendVideoAck(d2cbuffer &receivedDataBuffer, size_t bytes_received);

			void processAck(d2cbuffer &receivedDataBuffer, size_t bytes_transferred);

			void decodeNavdataPacket(d2cbuffer &receivedDataBuffer, std::size_t bytes_transferred);
			bool decodeVideoPacket(d2cbuffer &receivedDataBuffer, std::size_t bytes_transferred);

		    std::unique_ptr<boost::asio::ip::udp::socket> _d2c_socket = nullptr;
		    std::unique_ptr<boost::asio::ip::udp::socket> _c2d_socket = nullptr;
		    
		    std::unique_ptr<videodecoder> _videodecoder = nullptr;

		    boost::asio::ip::udp::endpoint _navdata_sender_endpoint;
    		d2cbuffer _navdata_receivedDataBuffer;

    		navdata _navdata;

    		// Acked command stuff
    		bool _ack_expected = false;
    		std::queue<uint8_t> _ack_seqNum_queue;
    		std::queue<navdata_id> _command_id_queue;
    		std::queue<std::vector<boost::any>> _command_arg_queue;
	};
}

#endif
