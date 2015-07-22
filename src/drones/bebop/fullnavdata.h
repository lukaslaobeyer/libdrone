#ifndef LIBDRONE_BEBOP_FULLNAVDATA_H
#define LIBDRONE_BEBOP_FULLNAVDATA_H

#include <types.h>
#include <drones/bebop/types.h>
#include <drones/bebop/constants.h>
#include <array>
#include <memory>

#include <boost/asio.hpp>

#define FULL_NAVDATA_MAX_SIZE 4096

namespace bebop
{
	class fullnavdata
	{
		public:
			fullnavdata();
			void init(std::string ip, boost::asio::io_service &io_service);

			std::shared_ptr<navdata> getNavdata();

		private:
			void navdataPacketReceived(const boost::system::error_code &error, std::size_t bytes_transferred);

			std::unique_ptr<boost::asio::ip::udp::socket> _navdata_socket = nullptr;
			boost::asio::ip::udp::endpoint endpoint;

			boost::asio::ip::udp::endpoint _navdata_sender_endpoint;

			std::array<char, FULL_NAVDATA_MAX_SIZE> _navdata_buf;

			bebop::navdata _navdata;

			bool _gotNavdata = false;

	};
}

#endif
