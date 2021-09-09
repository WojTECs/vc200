#pragma once
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>

#include <ctime>
#include <fstream>
#include <string>

#include "pid/pid.h"
#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/current_measurement.h"
#include "stalker_driver/motor_controller.h"
#include "std_srvs/SetBool.h"
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
  std::shared_ptr<Interface::UpstreamData::CurrentMeasurementFrame> currentUpstream;
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  std::vector<hardware_interface::JointHandle> getJoints();

  void sendCommand(Interface::DownstreamData::MovementCommandDataset cmd);

  void readData();
  void writeData();

 private:
  bool pidState_;
  PID leftChannelPid_, rightChannelPid_;
  bool pidServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  // char currentFileName[50];
  // std::ofstream currentLogFile;
  ros::ServiceServer pidService;
  joint_state leftJointState;
  joint_state rightJointState;
  double leftVelocityCommand;
  double rightVelocityCommand;
  hardware_interface::JointHandle leftJointHandle, rightJointHandle;

  double max_command;
  int encoder_resolution;
};
}  // namespace vc200_driver