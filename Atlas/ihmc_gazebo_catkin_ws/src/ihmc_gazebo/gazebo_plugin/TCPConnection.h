/*
 * TCPConnection.h
 *
 *  Created on: Feb 25, 2013
 *      Author: jesper
 */

#ifndef TCPCONNECTION_H_
#define TCPCONNECTION_H_

#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

using boost::asio::ip::tcp;


class TCPConnection
  : public boost::enable_shared_from_this<TCPConnection>
{
public:
  typedef boost::shared_ptr<TCPConnection> pointer;


  static pointer create(boost::asio::io_service& io_service, int bufferSize);

  tcp::socket& getSocket();

  void start();
  bool connected();
  void addReadListener(boost::function<void(char*,std::size_t)> callback);
  void send(boost::asio::streambuf& request);

  ~TCPConnection();
private:
  TCPConnection(boost::asio::io_service& io_service, int bufferSize);

  void startRead();
  void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred);

  int bufferSize;
  char* readBuffer;
  tcp::socket socket;
  boost::function<void(char*,std::size_t)> callback;
};

#endif /* TCPCONNECTION_H_ */
