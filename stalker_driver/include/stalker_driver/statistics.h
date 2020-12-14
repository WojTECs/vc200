#pragma once

#include "stalker_driver/UpstreamDataType.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
namespace Interface {
namespace UpstreamData {
struct StatisticsDataset {
  uint32_t sentPackages;
  uint32_t malformedPackages;
};
class PackageStatisticsFrame : public Interface::UpstreamDataType {
private:
  StatisticsDataset data;
  void doTheProcessing() override;

public:
  PackageStatisticsFrame();
  virtual ~PackageStatisticsFrame();
  void readData(StatisticsDataset& dest);
  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  // std::unique_ptr<Interface::UpstreamDataType> getClone()override;
};
}  // namespace UpstreamData
}  // namespace Interface
