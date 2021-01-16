#pragma once
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>

#include <string>

#include "pid/pid.h"
#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/motor_controller.h"

namespace vc200_driver {
struct joint_state {
  joint_state() : position(0.0), velocity(0.0), effort(0.0) {}
  double position;
  double velocity;
  double effort;
};
class MotorController {
 public:
  MotorController(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh);

  std::shared_ptr<Interface::DownstreamData::MovementOrderLeftRightFrame> motorDownstream;
  std::shared_ptr<Interface::UpstreamData::MovementInformationLeftRightFrame> motorUpstream;
  std::shared_ptr<Interface::UpstreamData::EncoderFrame> encUpstream;
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
  ros::NodeHandle nh_;

  // void setSpeeds(double left, double right);
  // void setLeftSpeed(double left);
  // void setRightSpeed(double right);

  void sendCommand(Interface::DownstreamData::MovementCommandDataset cmd);

  void readData();
  void writeData();

 private:
  PID leftChannelPid_, rightChannelPid_;

  hardware_interface::JointHandle leftJointHandle;
  joint_state leftJointState;
  joint_state rightJointState;
  double leftVelocityCommand;
  double rightVelocityCommand;
  hardware_interface::JointHandle rightJointHandle;
  std::string joint_left_name;
  double joint_left_max_command;
  int joint_left_encoder_resolution;
  std::string joint_right_name;
  double joint_right_max_command;
  int joint_right_encoder_resolution;
};
}  // namespace vc200_driver