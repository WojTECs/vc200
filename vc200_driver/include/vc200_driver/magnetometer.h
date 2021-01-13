#pragma once

#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/magnetometer.h"

namespace vc200_driver {
class Magnetometer {
 public:
  Magnetometer()
    : upstream(new Interface::UpstreamData::MagnetometerFrame)
    , downstream(new Interface::DownstreamData::MagnetometerFrame) {}

  std::shared_ptr<Interface::DownstreamData::MagnetometerFrame> downstream;
  std::shared_ptr<Interface::UpstreamData::MagnetometerFrame> upstream;
};
}  // namespace vc200_driver
