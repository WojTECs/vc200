#include "vc200_driver/motor_controller.h"

namespace vc200_driver {
MotorController::MotorController(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
  : motorDownstream(new Interface::DownstreamData::MovementOrderLeftRightFrame)
  , motorUpstream(new Interface::UpstreamData::MovementInformationLeftRightFrame)
  , encUpstream(new Interface::UpstreamData::EncoderFrame)
  , stClient_(st_if)
  , nh_(nh) {
  stClient_->addExpectedDataType(motorUpstream);
  stClient_->addExpectedDataType(encUpstream);
}
}  // namespace vc200_driver
