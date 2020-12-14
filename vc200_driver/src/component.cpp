#include "vc200_driver/component.h"
namespace vc200_driver {

Component::Component(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if,std::string _name) {
  stClient_ = st_if;
  name = _name;
}
}