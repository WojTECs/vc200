#pragma once

#include <ros/ros.h>

#include <string>

// #include "pid/pid.h"
#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/motor_controller.h"

namespace vc200_driver {
class MotorController {
 public:
  MotorController(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh);

 private:
  std::shared_ptr<Interface::DownstreamData::MovementOrderLeftRightFrame> motorDownstream;
  std::shared_ptr<Interface::UpstreamData::MovementInformationLeftRightFrame> motorUpstream;
  std::shared_ptr<Interface::UpstreamData::EncoderFrame> encUpstream;
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
  ros::NodeHandle nh_;
};
}  // namespace vc200_driver