#pragma once
#include "stalker_driver/STInterfaceClientUDP.h"
#include <string>
namespace vc200_driver {
class Component {
public:
  Component(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, std::string _name);
  // virtual ~Component();
  // virtual void readData() = 0;
  // virtual void writeData() = 0;
  std::string name;

public:
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
};
}  // namespace vc200_driver
