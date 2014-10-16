/*
 * TCPServer.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: jesper
 */

#include "TCPServer.h"
#include <boost/thread.hpp>
#include <pthread.h>

TCPServer::TCPThread::TCPThread(boost::asio::io_service& io_service) :
		io_service(io_service) {
}
void TCPServer::TCPThread::run() {
	try {
		io_service.run();
	} catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}
}

TCPServer::TCPServer(int port, int bufferSize) :
		io_service(), acceptor(io_service, tcp::endpoint(tcp::v4(), port)), tcpConnection(
				TCPConnection::create(acceptor.get_io_service(), bufferSize)) {

	acceptor.async_accept(tcpConnection->getSocket(),
			boost::bind(&TCPServer::handle_accept, this, tcpConnection,
					boost::asio::placeholders::error));

	TCPServer::TCPThread* tcpThread = new TCPServer::TCPThread(io_service);
	boost::thread thread(boost::bind(&TCPServer::TCPThread::run, tcpThread));

}

TCPServer::~TCPServer() {
	// TODO Auto-generated destructor stub
}

void TCPServer::send(boost::asio::streambuf& request) {
	if (tcpConnection->connected()) {
		tcpConnection->send(request);
	}
}

void TCPServer::handle_accept(TCPConnection::pointer tcpConnection,
		const boost::system::error_code& error) {
	if (!error) {
		tcpConnection->start();
	}
	else
	{
		std::cerr << error.message() << std::endl;
	}
}

void TCPServer::addReadListener(boost::function<void(char*,std::size_t)> callback) {
	tcpConnection->addReadListener(callback);
}



