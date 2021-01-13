#pragma once
#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/accelerometer.h"

namespace vc200_driver {
class Accelerometer {
 public:
  Accelerometer()
    : downstream(new Interface::DownstreamData::AccelerometerFrame)
    , upstream(new Interface::UpstreamData::AccelerometerFrame) {}

  std::shared_ptr<Interface::UpstreamData::AccelerometerFrame> upstream;
  std::shared_ptr<Interface::DownstreamData::AccelerometerFrame> downstream;
};
}  // namespace vc200_driver