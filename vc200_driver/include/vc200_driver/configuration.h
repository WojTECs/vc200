#pragma once
#include <ros/ros.h>

#include <string>

#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/configuration.h"

namespace vc200_driver {
class Configurator {
 public:
  Configurator(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if,
               ros::NodeHandle &nh);
  std::shared_ptr<Interface::DownstreamData::MainConfigurationFrame> downstream;

 private:
  ros::NodeHandle nh_;
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
};
}  // namespace vc200_driver