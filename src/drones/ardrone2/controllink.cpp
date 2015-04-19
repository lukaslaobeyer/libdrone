#include "controllink.h"
#include "atcommands/atcommand.h"
#include "atcommands/configcommand.h"
#include "atcommands/configidscommand.h"

using namespace std;
using boost::asio::ip::udp;

ControlLink::ControlLink()
{

}

ControlLink::~ControlLink()
{
	if(socket != nullptr)
	{
		delete socket;
	}
}

void ControlLink::init(std::string ip, boost::asio::io_service &io_service)
{
	udp::resolver resolver(io_service);
	udp::resolver::query query(udp::v4(), ip, to_string(ardrone2::CONTROL_PORT));
	udp::endpoint receiver_endpoint = *resolver.resolve(query);

	socket = new udp::socket(io_service);
	socket->open(udp::v4());
	socket->bind(udp::endpoint(udp::v4(), ardrone2::CONTROL_PORT));
	socket->connect(receiver_endpoint);
}

void ControlLink::setAppID()
{
	sendATCommands(vector<ATCommand>{ConfigIDSCommand(), ConfigCommand("custom:session_id", ardrone2::app_id::SESSION_ID)});
	sendATCommands(vector<ATCommand>{ConfigIDSCommand(), ConfigCommand("custom:profile_id", ardrone2::app_id::PROFILE_ID)});
	sendATCommands(vector<ATCommand>{ConfigIDSCommand(), ConfigCommand("custom:application_id", ardrone2::app_id::APPLICATION_ID)});
}

void ControlLink::sendATCommands(vector<ATCommand> cmds)
{
	if(socket)
	{
		stringstream buf;

		for(unsigned int i = 0; i < cmds.size(); i++)
		{
			buf << "AT*";
			buf << cmds[i].getCommand();
			buf << '=';
			buf << to_string(seqNum);

			for(unsigned int j = 0; j < cmds[i].getParameters().size(); j++)
			{
				buf << ',';
				buf << cmds[i].getParameters()[j];
			}

			buf << '\r';

			seqNum++;
		}

		socket->send(boost::asio::buffer(buf.str()));
	}
	else
	{
		throw ControlLinkNotInitializedException();
	}
}

void ControlLink::close()
{
	delete socket;
	socket = nullptr;
}
