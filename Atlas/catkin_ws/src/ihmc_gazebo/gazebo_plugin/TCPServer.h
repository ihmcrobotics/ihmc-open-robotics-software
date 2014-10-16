/*
 * TCPServer.h
 *
 *  Created on: Feb 25, 2013
 *      Author: jesper
 */

#ifndef TCPSERVER_H_
#define TCPSERVER_H_

#include <ctime>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "TCPConnection.h"

using boost::asio::ip::tcp;

class TCPServer {
private:
	class TCPThread {
	public:
		TCPThread(boost::asio::io_service& io_service);
		void run();
	private:
		boost::asio::io_service& io_service;
	};

	boost::asio::io_service io_service;
	tcp::acceptor acceptor;
	TCPConnection::pointer tcpConnection;

	void handle_accept(TCPConnection::pointer new_connection,
			const boost::system::error_code& error);
	void handleRead(char* buf);


public:
	TCPServer(int port, int bufferSize);
	void send(boost::asio::streambuf& request);
	void addReadListener(boost::function<void(char*,std::size_t)> callback);
	virtual ~TCPServer();
	void join();
	void run();
};

#endif /* TCPSERVER_H_ */
