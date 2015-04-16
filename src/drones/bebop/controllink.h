#ifndef LIBDRONE_BEBOP_CONTROLLINK_H
#define LIBDRONE_BEBOP_CONTROLLINK_H

#include <drones/bebop/constants.h>

#include <boost/asio.hpp>

namespace bebop
{
	class controllink
	{
		public:
			explicit controllink();
			void init(std::string ip, boost::asio::io_service &io_service);
	};
}

#endif
