#include "vc200_driver/component.h"
#include "stalker_driver/motor_controller.h"
#include <string>

namespace vc200_driver {
class MotorController : public Component {
public:
  MotorController(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if,std::string _name);

  std::shared_ptr<Interface::DownstreamData::MovementOrderLeftRightFrame> motorDownstream;
  std::shared_ptr<Interface::UpstreamData::MovementInformationLeftRightFrame> motorUpstream;
  std::shared_ptr<Interface::UpstreamData::EncoderFrame> encUpstream;
};
}  // namespace vc200_driver