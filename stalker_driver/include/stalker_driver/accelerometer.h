#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "stalker_driver/UpstreamDataType.h"
#include "stalker_driver/DownstreamDataType.h"
#include <stalker_driver/DEMA.h>
#include <memory>

namespace Interface
{
namespace UpstreamData
{
struct AccelerometerDataset
{
  float xAxis;
  float yAxis;
  float zAxis;
  uint32_t timestamp;
};
class AccelerometerFrame : public Interface::UpstreamDataType
{
private:
  DEMAFilter x, y, z;
  std::vector<AccelerometerDataset> datasets;
  AccelerometerDataset data;
  void filter();
  bool readyToRead;
  void doTheProcessing() override;
  void print();

public:
  void readData(AccelerometerDataset& d);
  AccelerometerFrame();
  virtual ~AccelerometerFrame();
  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  // std::unique_ptr<Interface::UpstreamDataType> getClone() override;
};
}  // namespace UpstreamData

namespace DownstreamData
{
struct AccelerometerCommandDataset
{
  int registryAddress;
  int registryValue;
};
class AccelerometerFrame : public Interface::DownstreamDataType
{
private:
  AccelerometerCommandDataset command;
  void doTheProcessing() override;

public:
  AccelerometerFrame();
  virtual ~AccelerometerFrame();

  std::vector<uint8_t> serialize() override;
  void setCommand(AccelerometerCommandDataset& in);
};

}  // namespace DownstreamData
}  // namespace Interface