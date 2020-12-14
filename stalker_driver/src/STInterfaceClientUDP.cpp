#include "stalker_driver/STInterfaceClientUDP.h"

#include <iostream>
#include <utility>
#include <algorithm>

#include <boost/asio.hpp>

STInterface::STInterfaceClientUDP::STInterfaceClientUDP(unsigned short iPort, std::string stAddress, std::string stPort)
  : stAddress(stAddress)
  , stPort(stPort)
  , resolver(tcpIoService)
  , udpSocket(ioService, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), iPort))
  , tcpSocket(tcpIoService)
  , query(boost::asio::ip::tcp::v4(), stAddress.c_str(), stPort)
  , endReceiving(false)
{
  iterator = resolver.resolve(query);
  try
  {
    boost::asio::connect(tcpSocket, iterator);
  }
  catch (const boost::exception& e)
  {
    std::string diag = diagnostic_information(e);
    std::cout << "ST Connection error. Boost says: " << diag << std::endl;
  }
  std::cout << "ST Interface Client created" << std::endl;
}

void STInterface::STInterfaceClientUDP::addExpectedDataType(
    std::shared_ptr<Interface::UpstreamDataType> iExpectedDataType)
{
  // expectedDataTypes.push_back(std::move(iExpectedDataType));
  if (expectedDataTypesRegistry[iExpectedDataType->getProtocolIdentificator()] != nullptr)
  {
    std::cout << "Error during enlisting new message type. This message ID is already occupied!" << std::endl;
  }
  else
  {
    expectedDataTypesRegistry[iExpectedDataType->getProtocolIdentificator()] = iExpectedDataType;
    std::cout << "\033[1;32mSTInterface session enlisted new data type\033[0m" << std::endl;
  }
}

void STInterface::STInterfaceClientUDP::run()
{
  doReceive();
  ioService.run();
}
void STInterface::STInterfaceClientUDP::stop()
{
  endReceiving = true;
}
void STInterface::STInterfaceClientUDP::doReceive()
{
  udpSocket.async_receive_from(
      boost::asio::buffer(rawSocketData, max_length), senderEndpoint,
      [this](boost::system::error_code error, std::size_t bytesTransferred) {
        if (!error && bytesTransferred > 0)
        {
          int byteProcessed = 0;

          while (byteProcessed < bytesTransferred && bytesTransferred >= 2)
          {
            // reading first byte of a batch describing message type
            uint8_t batchMessageType = rawSocketData[byteProcessed];
            byteProcessed++;

            // reading first byte of a batch describing batch length expressed by a number of bytes following
            uint8_t batchMessageLength = rawSocketData[byteProcessed];
            byteProcessed++;

            // pass vector substring of data(of a given type)
            if (byteProcessed + batchMessageLength > bytesTransferred)
            {
              std::cout << "\033[1;31mCRITICAL! STalker can't keep up with message processing, one frame is "
                           "lost.\033[0m"
                        << std::endl;
              break;
            }

            if (expectedDataTypesRegistry[batchMessageType] == nullptr)
            {
              std::cout << "Message buffer is getting full and one message got split apart, or bad message identifier "
                           "was received."
                        << std::endl;
              break;
            }

            expectedDataTypesRegistry[batchMessageType]->deserialize(
                rawSocketData + byteProcessed,
                batchMessageLength * expectedDataTypesRegistry[batchMessageType]->getSTBinarySize());

            byteProcessed += batchMessageLength * expectedDataTypesRegistry[batchMessageType]->getSTBinarySize();
            //                int b=0;
          }
          if (!endReceiving)
          {
            doReceive();
          }
        }
      });
}

void STInterface::STInterfaceClientUDP::publishData(Interface::DownstreamDataType& iData)
{  // non thread safe function
  boost::system::error_code error;
  boost::asio::write(tcpSocket, boost::asio::buffer(iData.serialize()), error);
  if (error)
  {
    std::cout << "ST tcp write error. Boost error message is: " << std::string(error.message()) << std::endl;
    iterator = resolver.resolve(query);
    try
    {
      boost::asio::connect(tcpSocket, iterator);
    }
    catch (const boost::exception& e)
    {
      std::string diag = diagnostic_information(e);
      std::cout << "ST Connection error. Boost says: " << diag << std::endl;
    }
  }
}
