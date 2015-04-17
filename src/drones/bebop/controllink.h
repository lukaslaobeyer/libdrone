#ifndef LIBDRONE_BEBOP_CONTROLLINK_H
#define LIBDRONE_BEBOP_CONTROLLINK_H

#include <drones/bebop/constants.h>

#include <memory>

#include <boost/asio.hpp>

namespace bebop
{
	class controllink
	{
		public:
			explicit controllink();
			
			void init(std::string ip, boost::asio::io_service &io_service);
			void startReceive();
			
		private:
		    std::unique_ptr<boost::asio::ip::udp::socket> _navdata_socket = nullptr;
		    
		    boost::asio::ip::udp::endpoint sender_endpoint;
    		char _receivedDataBuffer[4096];
	};
}

#endif
