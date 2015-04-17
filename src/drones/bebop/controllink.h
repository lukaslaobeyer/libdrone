#ifndef LIBDRONE_BEBOP_CONTROLLINK_H
#define LIBDRONE_BEBOP_CONTROLLINK_H

#include <drones/bebop/constants.h>

#include <memory>
#include <array>

#include <boost/asio.hpp>

namespace bebop
{
	class controllink
	{
		public:
			explicit controllink();
			
			void init(std::string ip, boost::asio::io_service &io_service);
			
		private:
			void startReceivingNavdata();
			void navdataPacketReceived(const boost::system::error_code &error, std::size_t bytes_transferred);

		    std::unique_ptr<boost::asio::ip::udp::socket> _navdata_socket = nullptr;
		    
		    boost::asio::ip::udp::endpoint _navdata_sender_endpoint;
    		std::array<char, 4096> _navdata_receivedDataBuffer;
	};
}

#endif
