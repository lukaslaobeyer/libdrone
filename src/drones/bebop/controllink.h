#ifndef LIBDRONE_BEBOP_CONTROLLINK_H
#define LIBDRONE_BEBOP_CONTROLLINK_H

#define BEBOP_NAVDATA_BUFFER_SIZE 4096

#include <drones/bebop/constants.h>
#include <types.h>
#include <commands.h>

#include <memory>
#include <array>
#include <vector>

#include <boost/any.hpp>

#include <boost/asio.hpp>

class InvalidCommandException : public std::runtime_error
{
	public:
		InvalidCommandException() : std::runtime_error("Invalid command (arguments do not match command requirements)") { }
};

namespace bebop
{
	typedef std::array<char, BEBOP_NAVDATA_BUFFER_SIZE> d2cbuffer;
	typedef std::array<char, 7> frameheaderbuf;

	class controllink
	{
		public:
			explicit controllink();
			
			void init(std::string ip, boost::asio::io_service &io_service);
			
			void sendCommand(navdata_id &command_id, std::vector<boost::any> &args);

			static std::vector<char> createCommand(navdata_id &command_id, std::vector<boost::any> &args);
		private:
			void startReceivingNavdata();
			void navdataPacketReceived(const boost::system::error_code &error, std::size_t bytes_transferred);

			void sendPong(d2cbuffer receivedDataBuffer, std::size_t bytes_transferred);

			void decodeNavdataPacket(d2cbuffer receivedDataBuffer, std::size_t bytes_transferred);

			frameheaderbuf createHeader(frameheader &header);

		    std::unique_ptr<boost::asio::ip::udp::socket> _d2c_socket = nullptr;
		    std::unique_ptr<boost::asio::ip::udp::socket> _c2d_socket = nullptr;
		    
		    boost::asio::ip::udp::endpoint _navdata_sender_endpoint;
    		d2cbuffer _navdata_receivedDataBuffer;
	};
}

#endif
