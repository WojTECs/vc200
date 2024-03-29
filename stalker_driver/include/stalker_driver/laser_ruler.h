#pragma once

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"

namespace Interface {
namespace UpstreamData {
struct LaserRulerDataset {
  float scan[8];
};
class LaserRulerFrame : public UpstreamDataType {
 private:
  LaserRulerDataset data;
  // void filter();
  void doTheProcessing() override;

 public:
  LaserRulerFrame();
  virtual ~LaserRulerFrame();
  void readData(LaserRulerDataset& dest);
  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  void handleSavingData(float &data, uint8_t iDataStream);
};
}  // namespace UpstreamData
}  // namespace Interface
