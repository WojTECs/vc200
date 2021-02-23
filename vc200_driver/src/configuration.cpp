#include "vc200_driver/configuration.h"

namespace vc200_driver {
Configurator::Configurator(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
  : downstream(new Interface::DownstreamData::MainConfigurationFrame), stClient_(st_if), nh_(nh) {}
}  // namespace vc200_driver