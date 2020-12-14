#pragma once
#include "vc200_driver/component.h"
#include "stalker_driver/accelerometer.h"

namespace vc200_driver {
class Accelerometer {
public:
  Accelerometer()
    : downstream(new Interface::DownstreamData::AccelerometerFrame)
    , upstream(new Interface::UpstreamData::AccelerometerFrame) {
  }

  std::shared_ptr<Interface::UpstreamData::AccelerometerFrame> upstream;
  std::shared_ptr<Interface::DownstreamData::AccelerometerFrame> downstream;
};
}  // namespace vc200_driver