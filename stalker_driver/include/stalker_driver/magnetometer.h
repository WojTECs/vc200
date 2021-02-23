#pragma once

#include <stalker_driver/DEMA.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"

namespace Interface {
namespace DownstreamData {
struct MagnetometerCommandDataset {
  int registryAddress;
  int registryValue;
};
class MagnetometerFrame : public Interface::DownstreamDataType {
 private:
  void doTheProcessing() override;
  MagnetometerCommandDataset command;

 public:
  MagnetometerFrame();
  virtual ~MagnetometerFrame();

  std::vector<uint8_t> serialize() override;
  void setCommnad(MagnetometerCommandDataset& in);
  // void deserialize(const uint16_t* msgArray, uint16_t arraySize) override;
};

}  // namespace DownstreamData
}  // namespace Interface

namespace Interface {
namespace UpstreamData {
struct MagnetometerDataset {
  float xAxis;
  float yAxis;
  float zAxis;
  uint32_t timestamp;
};

class MagnetometerFrame : public Interface::UpstreamDataType {
 private:
  DEMAFilter x, y, z;

  std::vector<MagnetometerDataset> datasets;
  MagnetometerDataset data;
  void filter();
  void doTheProcessing() override;

 public:
  void readData(MagnetometerDataset& dest);

  MagnetometerFrame();
  virtual ~MagnetometerFrame();

  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  // hard_iron
  float offset_x;
  float offset_y;
  float offset_z;
  // soft_iron
  float scale_x;
  float scale_y;
  float scale_z;
  // std::unique_ptr<Interface::UpstreamDataType> getClone() override;
};
}  // namespace UpstreamData
}  // namespace Interface
