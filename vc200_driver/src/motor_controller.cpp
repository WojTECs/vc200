#include "vc200_driver/motor_controller.h"

namespace vc200_driver
{
MotorController::MotorController(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, std::string _name)
  : motorDownstream(new Interface::DownstreamData::MovementOrderLeftRightFrame)
  , motorUpstream(new Interface::UpstreamData::MovementInformationLeftRightFrame)
  , encUpstream(new Interface::UpstreamData::EncoderFrame)
  , Component(st_if, _name)
{
  stClient_->addExpectedDataType(motorUpstream);
  stClient_->addExpectedDataType(encUpstream);
}
}  // namespace vc200_driver
