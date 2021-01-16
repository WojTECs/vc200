#include "vc200_driver/motor_controller.h"

namespace vc200_driver {
MotorController::MotorController(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
    : motorDownstream(new Interface::DownstreamData::MovementOrderLeftRightFrame)
    , motorUpstream(new Interface::UpstreamData::MovementInformationLeftRightFrame)
    , encUpstream(new Interface::UpstreamData::EncoderFrame)
    , stClient_(st_if)
    , nh_(nh)
    , leftChannelPid_(nh, "pid/left")
    , rightChannelPid_(nh, "pid/right") {
  stClient_->addExpectedDataType(motorUpstream);
  stClient_->addExpectedDataType(encUpstream);

  priv_nh_ = ros::NodeHandle(nh_, "motor_contoller");
  joint_left_name = "left_wheel";
  if (!priv_nh_.getParam("joint/left/name", joint_left_name)) {
    ROS_WARN_STREAM("Can not find name of left joint, default: " << joint_left_name);
  }

  joint_left_max_command = 10.0;
  if (!priv_nh_.getParam("joint/left/max_command", joint_left_max_command)) {
    ROS_WARN_STREAM("Can not find max command of left joint, default: " << joint_left_max_command);
  }

  joint_left_encoder_resolution = 100;
  if (!priv_nh_.getParam("joint/left/encoder_resolution", joint_left_encoder_resolution)) {
    ROS_WARN_STREAM("Can not find encoder resolution of left joint, default: " << joint_left_encoder_resolution);
  }

  joint_right_name = "right_wheel";
  if (!priv_nh_.getParam("joint/right/name", joint_right_name)) {
    ROS_WARN_STREAM("Can not find name of right joint, default: " << joint_right_name);
  }

  joint_right_max_command = 10.0;
  if (!priv_nh_.getParam("joint/right/max_command", joint_right_max_command)) {
    ROS_WARN_STREAM("Can not find max command of right joint, default: " << joint_right_max_command);
  }

  joint_right_encoder_resolution = 100;
  if (!priv_nh_.getParam("joint/right/encoder_resolution", joint_right_encoder_resolution)) {
    ROS_WARN_STREAM("Can not find encoder resolution of right joint, default: " << joint_right_encoder_resolution);
  }

  hardware_interface::JointStateHandle state_left_handle(joint_left_name, &leftJointState.position,
                                                         &leftJointState.velocity, &leftJointState.effort);

  hardware_interface::JointHandle left_handle(state_left_handle, &leftVelocityCommand);
  // leftJointHandle(state_left_handle, &command_[i]);
  leftJointHandle = left_handle;

  hardware_interface::JointStateHandle state_right_handle(joint_right_name, &rightJointState.position,
                                                          &rightJointState.velocity, &rightJointState.effort);

  hardware_interface::JointHandle right_handle(state_right_handle, &rightVelocityCommand);
  // leftJointHandle(state_left_handle, &command_[i]);
  rightJointHandle = right_handle;

  // command timeout
}

void MotorController::sendCommand(Interface::DownstreamData::MovementCommandDataset cmd) {
  motorDownstream->setCommand(cmd);
  stClient_->publishData(*motorDownstream);
  // cmd.
}

void MotorController::readData() {
  Interface::UpstreamData::MovementInformationDataset motorData;
  Interface::UpstreamData::EncoderDataset encData;
  motorUpstream->readData(motorData);
  encUpstream->readData(encData);
  leftChannelPid_.setPoint(leftVelocityCommand);
  rightChannelPid_.setPoint(rightVelocityCommand);
  leftChannelPid_.update(encData.leftSideVelocity.value);
  rightChannelPid_.update(encData.rightSideVelocity.value);

  leftJointState.effort = motorData.leftWheelPwm;
  leftJointState.velocity = encData.leftSideVelocity.value;
  leftJointState.position = (encData.leftSideDistance.int_value / joint_left_encoder_resolution) * 2.0 * M_PI;

  rightJointState.effort = motorData.rightWheelPwm;
  rightJointState.velocity = encData.rightSideVelocity.value;
  rightJointState.position = (encData.rightSideDistance.int_value / joint_right_encoder_resolution) * 2.0 * M_PI;
}
void MotorController::writeData() {
  Interface::DownstreamData::MovementCommandDataset cmd;

  cmd.leftSidePWM = leftChannelPid_.getControll();
  cmd.rightSidePWM = rightChannelPid_.getControll();
  if (cmd.leftSidePWM != 0.0) {
    cmd.leftDirection = (leftChannelPid_.getPoint() > 0) ? 1 : 2;
  } else {
    cmd.leftDirection = 0;
  }

  if (cmd.rightSidePWM != 0.0) {
    cmd.rightDirection = (rightChannelPid_.getPoint() > 0) ? 1 : 2;
  } else {
    cmd.rightDirection = 0;
  }

  cmd.shallQueue = 0;
  cmd.timeToDrive = 100;

  motorDownstream->setCommand(cmd);
}
// void MotorController::setLeftSpeed(double speed) { leftChannelPid_.setPoint(speed); }

// void MotorController::setRightSpeed(double speed) { rightChannelPid_.setPoint(speed); }

// void MotorController::setSpeeds(double left, double right) {
//   setLeftSpeed(left);
//   setRightSpeed(right);
// }

std::vector<hardware_interface::JointHandle> MotorController::getJoints() {
  std::vector<hardware_interface::JointHandle> output;
  output.push_back(leftJointHandle);
  output.push_back(rightJointHandle);
  return output;
}
}  // namespace vc200_driver
