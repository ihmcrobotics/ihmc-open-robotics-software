/*
 * TCPConnection.cc
 *
 *  Created on: Feb 25, 2013
 *      Author: jesper
 */

#include "TCPConnection.h"

TCPConnection::TCPConnection(boost::asio::io_service& io_service, int bufferSize) :
		bufferSize(bufferSize), readBuffer(new char[bufferSize]), socket(io_service), callback(0) {
}


TCPConnection::~TCPConnection()
{
	delete [] readBuffer;
}

TCPConnection::pointer TCPConnection::create(
		boost::asio::io_service& io_service, int bufferSize) {
	return pointer(new TCPConnection(io_service, bufferSize));
}

tcp::socket& TCPConnection::getSocket() {
	return socket;
}

void TCPConnection::start() {
	socket.set_option(tcp::no_delay(true));	// Disable Nagle's algorithm
	startRead();
}

bool TCPConnection::connected() {
	return socket.is_open();
}

void TCPConnection::startRead() {
	memset(readBuffer, 0, bufferSize * sizeof(char));
	boost::asio::async_read(socket, boost::asio::buffer(readBuffer, bufferSize),
			boost::asio::transfer_at_least(1),
			boost::bind(&TCPConnection::handle_read, shared_from_this(),
					boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void TCPConnection::send(boost::asio::streambuf& request) {
	try {
		boost::asio::write(socket, request);
	} catch (std::exception& e) {
		std::cerr << "Connection lost" << std::endl;
	}
}

void TCPConnection::addReadListener(boost::function<void(char*, std::size_t)> callback) {
	TCPConnection::callback = callback;
}

void TCPConnection::handle_read(const boost::system::error_code& error, std::size_t bytes_transferred) {
	if (!error) {
		if (callback) {
			callback(readBuffer, bytes_transferred);
		}
		startRead();
	}
	else
	{
		std::cerr << "Connection closed" << std::endl;
		socket.close();
	}
}

