#include "vc200_driver/motor_controller.h"

namespace vc200_driver {
MotorController::MotorController(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
  : motorDownstream(new Interface::DownstreamData::MovementOrderLeftRightFrame)
  , motorUpstream(new Interface::UpstreamData::MovementInformationLeftRightFrame)
  , encUpstream(new Interface::UpstreamData::EncoderFrame)
  , currentUpstream(new Interface::UpstreamData::CurrentMeasurementFrame)
  , stClient_(st_if)
  , nh_(nh)
  , leftChannelPid_(nh, "pid/left")
  , rightChannelPid_(nh, "pid/right")
  , pidState_(true) {
  stClient_->addExpectedDataType(motorUpstream);
  stClient_->addExpectedDataType(encUpstream);
  stClient_->addExpectedDataType(currentUpstream);

  priv_nh_ = ros::NodeHandle(nh_, "motor_contoller");
  pidService = priv_nh_.advertiseService("turn_off_PID", &MotorController::pidServiceCallback, this);

  std::string joint_left_name;
  joint_left_name = "left_wheel";
  if (!priv_nh_.getParam("joint/left/name", joint_left_name)) {
    ROS_WARN_STREAM("Can not find name of left joint, default: " << joint_left_name);
  }

  std::string joint_right_name;
  joint_right_name = "right_wheel";
  if (!priv_nh_.getParam("joint/right/name", joint_right_name)) {
    ROS_WARN_STREAM("Can not find name of right joint, default: " << joint_right_name);
  }

  max_command = 100.0;
  if (!priv_nh_.getParam("max_command", max_command)) {
    ROS_WARN_STREAM("Can not find max command of joints, default: " << max_command);
  }

  encoder_resolution = 256;
  if (!priv_nh_.getParam("encoders_resolution", encoder_resolution)) {
    ROS_WARN_STREAM("Can not find encoder resolution of joints, default: " << encoder_resolution);
  }

  hardware_interface::JointStateHandle left_joint_state_handle(joint_left_name, &leftJointState.position,
                                                               &leftJointState.velocity, &leftJointState.effort);

  hardware_interface::JointHandle left_joint_handle(left_joint_state_handle, &leftVelocityCommand);
  leftJointHandle = left_joint_handle;

  hardware_interface::JointStateHandle right_joint_state_handle(joint_right_name, &rightJointState.position,
                                                                &rightJointState.velocity, &rightJointState.effort);

  hardware_interface::JointHandle right_joint_handle(right_joint_state_handle, &rightVelocityCommand);
  rightJointHandle = right_joint_handle;

  // time_t rawtime;
  // struct tm *timeinfo;
  // time(&rawtime);
  // timeinfo = localtime(&rawtime);
  // strftime(currentFileName, sizeof(currentFileName), "current log %d-%m-%Y %H:%M:%S", timeinfo);
  // currentLogFile.open(currentFileName, std::ios::app | std::ios::out | std::ios::binary);
  // if (currentLogFile.is_open()) {
  //   currentLogFile.close();
  //   std::cout << "Created file with current extended log with name: " << currentFileName << std::endl;
  // }

  // command timeout
}
bool MotorController::pidServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  if (req.data) {
    if (pidState_) {
      pidState_ = false;
      res.message = "PID turned off";
      res.success = true;
    } else {
      res.message = "PID already turned off";
      res.success = true;
    }
  } else {
    if (!pidState_) {
      pidState_ = true;
      res.message = "PID turned on";
      res.success = true;
    } else {
      res.message = "PID already turned on";
      res.success = true;
    }
  }

  return true;
}
void MotorController::sendCommand(Interface::DownstreamData::MovementCommandDataset cmd) {
  motorDownstream->setCommand(cmd);
  stClient_->publishData(*motorDownstream);
  // cmd.
}

void MotorController::readData() {
  Interface::UpstreamData::MovementInformationDataset motorData;
  Interface::UpstreamData::EncoderDataset encData;
  Interface::UpstreamData::CurrentMeasurementDataset currentData;
  std::vector<Interface::UpstreamData::CurrentMeasurementDataset> currentVector;
  motorUpstream->readData(motorData);
  encUpstream->readData(encData);
  currentUpstream->readData(currentData, currentVector, leftJointState.velocity, rightJointState.velocity,
                            leftJointState.position, rightJointState.position);

  // currentLogFile.open(currentFileName, std::ios::app | std::ios::out | std::ios::binary);
  // uint64_t timestamp = ros::Time::now().toNSec();
  // currentLogFile.write((char*)&timestamp, sizeof(timestamp));
  // size_t size = currentVector.size();
  // currentLogFile.write((char*)&size, sizeof(size));
  // currentLogFile.write((char*)&currentVector[0], currentVector.size() *
  // sizeof(Interface::UpstreamData::CurrentMeasurementDataset)); currentLogFile.close();

  leftChannelPid_.setPoint(leftVelocityCommand);
  rightChannelPid_.setPoint(rightVelocityCommand);
  leftChannelPid_.update(encData.leftSideVelocity.value);
  rightChannelPid_.update(encData.rightSideVelocity.value);

  leftJointState.effort = currentData.channel_A;
  leftJointState.velocity = encData.leftSideVelocity.value;
  leftJointState.position = (encData.leftSideDistance.int_value / (float)encoder_resolution) * 2.0 * M_PI;

  rightJointState.effort = currentData.channel_B;
  rightJointState.velocity = encData.rightSideVelocity.value;
  rightJointState.position = (encData.rightSideDistance.int_value / (float)encoder_resolution) * 2.0 * M_PI;
}

void MotorController::writeData() {
  Interface::DownstreamData::MovementCommandDataset cmd;
  if (pidState_) {
    cmd.leftSidePWM = leftChannelPid_.getControll();
    cmd.rightSidePWM = rightChannelPid_.getControll();

    if (cmd.leftSidePWM != 0.0) {
      cmd.leftDirection = (leftChannelPid_.getPoint() > 0) ? 1 : 2;
      cmd.leftSidePWM = abs(cmd.leftSidePWM);
    } else {
      cmd.leftDirection = 0;
    }

    if (cmd.rightSidePWM != 0.0) {
      cmd.rightDirection = (rightChannelPid_.getPoint() > 0) ? 1 : 2;
      cmd.rightSidePWM = abs(cmd.rightSidePWM);
    } else {
      cmd.rightDirection = 0;
    }

  } else {
    cmd.leftSidePWM = leftVelocityCommand;
    cmd.rightSidePWM = rightVelocityCommand;

    if (cmd.leftSidePWM != 0.0) {
      cmd.leftDirection = (leftVelocityCommand > 0) ? 1 : 2;
      cmd.leftSidePWM = abs(cmd.leftSidePWM);
    } else {
      cmd.leftDirection = 0;
    }

    if (cmd.rightSidePWM != 0.0) {
      cmd.rightDirection = (rightVelocityCommand > 0) ? 1 : 2;
      cmd.rightSidePWM = abs(cmd.rightSidePWM);
    } else {
      cmd.rightDirection = 0;
    }
  }
  // std::cout << "\x1B[2J\x1B[H";
  // std::cout << "cmd.leftSidePWM: " << cmd.leftSidePWM << std::endl;
  // std::cout << "cmd.rightSidePWM: " << cmd.rightSidePWM << std::endl;
  // std::cout << "leftVelocityCommand: " << leftVelocityCommand << std::endl;
  // std::cout << "rightVelocityCommand: " << rightVelocityCommand << std::endl;

  cmd.shallQueue = 0;
  cmd.timeToDrive = 500;

  sendCommand(cmd);
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
