#include "TCPServer.h"


#include <boost/date_time/time_duration.hpp>

void echoCallback(char* buffer)
{
	std::cout << buffer << std::endl;
}

int main() {
	std::cout << "Entering main" << std::endl;
	try {
		TCPServer server(1234, 5000);
		server.addReadListener(boost::bind(&echoCallback, _1));
		while(true)
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
			boost::asio::streambuf request;
			std::ostream request_stream(&request);
			request_stream << "TEST";

			server.send(request);
		}
	} catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}
	return 0;
}
