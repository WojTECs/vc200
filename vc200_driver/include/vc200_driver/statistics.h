#pragma once

#include <ros/ros.h>

#include <string>

#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/statistics.h"

namespace vc200_driver {
class Statistics {
 public:
  Statistics(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh);
  std::shared_ptr<Interface::UpstreamData::PackageStatisticsFrame> upstream;

 private:
  ros::NodeHandle nh_;
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
};
}  // namespace vc200_driver
