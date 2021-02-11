#pragma once

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"

namespace Interface {
namespace UpstreamData {
struct LaserRulerDataset {
  float scan[4];
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
};
}  // namespace UpstreamData
}  // namespace Interface