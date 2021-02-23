#pragma once

#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/gyroscope.h"

namespace vc200_driver {
class Gyroscope {
 public:
  Gyroscope()
      : upstream(new Interface::UpstreamData::GyroscopeFrame)
      , downstream(new Interface::DownstreamData::GyroscopeFrame){};

  std::shared_ptr<Interface::UpstreamData::GyroscopeFrame> upstream;
  std::shared_ptr<Interface::DownstreamData::GyroscopeFrame> downstream;
};
}  // namespace vc200_driver
