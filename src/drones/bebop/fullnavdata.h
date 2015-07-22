#ifndef LIBDRONE_BEBOP_FULLNAVDATA_H
#define LIBDRONE_BEBOP_FULLNAVDATA_H

namespace bebop
{
	class fullnavdata
	{
		public:
			fullnavdata();
			void init(std::string ip, boost::asio::io_service &io_service);
			bool tryConnecting();

			std::shared_ptr<navdata> getNavdata();

		private:


	};
}

#endif
