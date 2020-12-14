#include "vc200_driver/configuration.h"

namespace vc200_driver {
Configurator::Configurator(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if,std::string _name)
  : downstream(new Interface::DownstreamData::MainConfigurationFrame), Component(st_if,_name) {
}
}  // namespace vc200_driver