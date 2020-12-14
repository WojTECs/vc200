#pragma once

#include <vector>
#include <memory>
#include <atomic>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"

namespace Interface
{
class UpstreamDataType;
}

namespace STInterface
{
class Session;

class STInterfaceClientUDP
{
private:
  std::shared_ptr<Interface::UpstreamDataType> expectedDataTypesRegistry[256];

  boost::asio::io_service tcpIoService;
  boost::asio::ip::tcp::resolver resolver;
  boost::asio::ip::tcp::socket tcpSocket;
  boost::asio::ip::tcp::resolver::query query;
  boost::asio::ip::tcp::resolver::iterator iterator;

  boost::asio::io_service ioService;

  std::string stAddress;  // STM ip address
  std::string stPort;     // STM receiving port
  bool endReceiving;

public:
  enum
  {
    max_length = 1024
  };
  uint8_t rawSocketData[max_length];
  boost::asio::ip::udp::socket udpSocket;
  boost::asio::ip::udp::endpoint senderEndpoint;
  void doReceive();
  void doSend(std::size_t length);

  // Adds >>REFERENCE<< for the object in the internal list of expected data types.
  void addExpectedDataType(std::shared_ptr<Interface::UpstreamDataType> iExpectedDataType);
  void run();
  void stop();
  void publishData(Interface::DownstreamDataType& iData);

  STInterfaceClientUDP(unsigned short iPort, std::string stAddress, std::string stPort);
  virtual ~STInterfaceClientUDP()
  {
  }
};

}  // namespace STInterface
