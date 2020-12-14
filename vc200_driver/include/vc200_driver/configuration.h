#include "vc200_driver/component.h"
#include "stalker_driver/configuration.h"

namespace vc200_driver {
class Configurator : public Component {
public:
  Configurator(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if,std::string _name);
  std::shared_ptr<Interface::DownstreamData::MainConfigurationFrame> downstream;
};
}  // namespace vc200_driver