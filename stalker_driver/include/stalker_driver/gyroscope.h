#pragma once

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <stalker_driver/DEMA.h>

namespace Interface {
namespace DownstreamData {
struct GyroscopeCommandDataset {
  int registryAddress;
  int registryValue;
};
class GyroscopeFrame : public Interface::DownstreamDataType {
private:
  void doTheProcessing() override;
  GyroscopeCommandDataset command;

public:
  GyroscopeFrame();
  virtual ~GyroscopeFrame();
  void setCommand(GyroscopeCommandDataset& in);
  std::vector<uint8_t> serialize() override;
  // void deserialize(const uint16_t* msgArray, uint16_t arraySize) override;
};

}  // namespace DownstreamData

namespace UpstreamData {
struct GyroscopeDataset {
  float xAxis;
  float yAxis;
  float zAxis;
  uint32_t timestamp;
};
class GyroscopeFrame : public Interface::UpstreamDataType {
private:
  DEMAFilter x, y, z;

  std::vector<GyroscopeDataset> datasets;
  void filter();
  GyroscopeDataset data;
  void doTheProcessing() override;

public:
  GyroscopeFrame();
  virtual ~GyroscopeFrame();
  void readData(GyroscopeDataset& dest);
  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  // std::unique_ptr<Interface::UpstreamDataType> getClone() override;
};
}  // namespace UpstreamData
}  // namespace Interface
