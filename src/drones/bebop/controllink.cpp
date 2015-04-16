#include "controllink.h"

#include <array>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace bebop;

using namespace std;
using boost::asio::ip::tcp;

controllink::controllink()
{

}

void controllink::init(string ip, boost::asio::io_service &io_service)
{
	// Bebop discovery connection
	tcp::resolver resolver(io_service);
	tcp::resolver::query query(ip, to_string(bebop::DISCOVERY_PORT));
	tcp::endpoint receiver_endpoint = *resolver.resolve(query);

	tcp::socket socket(io_service);
	socket.open(tcp::v4());
	socket.bind(tcp::endpoint(tcp::v4(), bebop::DISCOVERY_PORT));
	socket.connect(receiver_endpoint);

	// Bebop configuration packet
	boost::property_tree::ptree discovery_request;
	discovery_request.put("controller_type", bebop::discovery::controller_type);
	discovery_request.put("controller_name", bebop::discovery::controller_name);
	discovery_request.put("d2c_port", bebop::D2C_PORT);

	// As JSON
	ostringstream discovery_request_buf;
	boost::property_tree::write_json(discovery_request_buf, discovery_request, false);

	socket.send(boost::asio::buffer(discovery_request_buf.str()));

	// Read response (should contain c2d_port ["controller-to-drone"] and other misc. information
	stringstream discovery_response_buf;
	for(;;)
	{
		array<char, 512> buf;
		boost::system::error_code error;

		size_t len = socket.read_some(boost::asio::buffer(buf), error);

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
	cout << "C2D port: " << c2d_port << endl;
}
