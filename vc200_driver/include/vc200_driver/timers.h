#pragma once

#include <ros/ros.h>

#include <string>

#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/timers.h"

namespace vc200_driver {
class Timers {
 public:
  Timers(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh);

 private:
  std::shared_ptr<Interface::DownstreamData::TimerConfigurationFrame> timerConfig;
  std::shared_ptr<Interface::UpstreamData::ServiceTimeFrame> serviceTimerUpstream;
  std::shared_ptr<Interface::UpstreamData::TimersFrame> timersUpstream;
  std::shared_ptr<Interface::UpstreamData::TimeSyncFrame> timeSyncUpstream;
  ros::NodeHandle nh_;
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
};
}  // namespace vc200_driver
