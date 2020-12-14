#pragma once

#include <iostream>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <mutex>

namespace Interface {
class UpstreamDataType {
private:
protected:
  uint8_t protocolIndentificator;
  std::size_t datasetBinarySize;
  std::mutex dataMutex;
  virtual void doTheProcessing() = 0;

public:
  uint8_t getProtocolIdentificator() const {
    return protocolIndentificator;
  }
  std::size_t getSTBinarySize() const {
    return datasetBinarySize;
  }

  virtual void deserialize(const uint8_t* iDataStream, const int iDataSize) = 0;
  // virtual std::unique_ptr<Interface::UpstreamDataType> getClone() = 0;
  virtual ~UpstreamDataType() {
  }
};

}  // namespace Interface
