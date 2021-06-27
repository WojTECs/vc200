#include "vc200_driver/motor_controller.h"

namespace vc200_driver
{
  MotorController::MotorController(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
      : motorDownstream(new Interface::DownstreamData::MovementOrderLeftRightFrame),
        motorUpstream(new Interface::UpstreamData::MovementInformationLeftRightFrame),
        encUpstream(new Interface::UpstreamData::EncoderFrame),
        currentUpstream(new Interface::UpstreamData::CurrentMeasurementFrame),
        stClient_(st_if), nh_(nh), leftChannelPid_(nh, "pid/left"), rightChannelPid_(nh, "pid/right")
  {
    stClient_->addExpectedDataType(motorUpstream);
    stClient_->addExpectedDataType(encUpstream);
    stClient_->addExpectedDataType(currentUpstream);

    priv_nh_ = ros::NodeHandle(nh_, "motor_contoller");

    std::string joint_front_left_name;
    joint_front_left_name = "front_left_wheel";
    if (!priv_nh_.getParam("joint/front_left/name", joint_front_left_name))
    {
      ROS_WARN_STREAM("Can not find name of front left joint, default: " << joint_front_left_name);
    }

    std::string joint_front_right_name;
    joint_front_right_name = "front_right_wheel";
    if (!priv_nh_.getParam("joint/front_right/name", joint_front_right_name))
    {
      ROS_WARN_STREAM("Can not find name of front right joint, default: " << joint_front_right_name);
    }

    std::string joint_rear_left_name;
    joint_rear_left_name = "rear_left_wheel";
    if (!priv_nh_.getParam("joint/rear_left/name", joint_rear_left_name))
    {
      ROS_WARN_STREAM("Can not find name of rear left joint, default: " << joint_rear_left_name);
    }
    std::string joint_rear_right_name;
    joint_rear_right_name = "rear_right_wheel";
    if (!priv_nh_.getParam("joint/rear_right/name", joint_rear_right_name))
    {
      ROS_WARN_STREAM("Can not find name of rear right joint, default: " << joint_rear_right_name);
    }

    max_command = 100.0;
    if (!priv_nh_.getParam("max_command", max_command))
    {
      ROS_WARN_STREAM("Can not find max command of left joint, default: " << max_command);
    }

    encoder_resolution = 256;
    if (!priv_nh_.getParam("encoders_resolution", encoder_resolution))
    {
      ROS_WARN_STREAM("Can not find encoder resolution of left joint, default: " << encoder_resolution);
    }

    hardware_interface::JointStateHandle state_front_left_handle(joint_front_left_name, &leftJointState.position,
                                                                 &leftJointState.velocity, &leftJointState.effort);

    hardware_interface::JointHandle front_left_handle(state_front_left_handle, &leftVelocityCommand);
    frontLeftJointHandle = front_left_handle;

    hardware_interface::JointStateHandle state_rear_left_handle(joint_rear_left_name, &leftJointState.position,
                                                                &leftJointState.velocity, &leftJointState.effort);

    hardware_interface::JointHandle rear_left_handle(state_rear_left_handle, &leftVelocityCommand);
    rearLeftJointHandle = rear_left_handle;

    hardware_interface::JointStateHandle state_front_right_handle(joint_front_right_name, &rightJointState.position,
                                                                  &rightJointState.velocity, &rightJointState.effort);

    hardware_interface::JointHandle front_right_handle(state_front_right_handle, &rightVelocityCommand);
    frontRightJointHandle = front_right_handle;

    hardware_interface::JointStateHandle state_rear_right_handle(joint_rear_right_name, &rightJointState.position,
                                                                 &rightJointState.velocity, &rightJointState.effort);

    hardware_interface::JointHandle rear_right_handle(state_rear_right_handle, &rightVelocityCommand);
    rearRightJointHandle = rear_right_handle;

    // command timeout
  }

  void MotorController::sendCommand(Interface::DownstreamData::MovementCommandDataset cmd)
  {
    motorDownstream->setCommand(cmd);
    stClient_->publishData(*motorDownstream);
    // cmd.
  }

  void MotorController::readData()
  {
    Interface::UpstreamData::MovementInformationDataset motorData;
    Interface::UpstreamData::EncoderDataset encData;
    Interface::UpstreamData::CurrentMeasurementDataset currentData;
    std::vector<Interface::UpstreamData::CurrentMeasurementDataset> currentVector;
    motorUpstream->readData(motorData);
    encUpstream->readData(encData);

    currentUpstream->readData(currentData, currentVector, leftJointState.velocity, rightJointState.velocity,
                              leftJointState.position, rightJointState.position);
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

  void MotorController::writeData()
  {
    Interface::DownstreamData::MovementCommandDataset cmd;

    cmd.leftSidePWM = leftChannelPid_.getControll();
    cmd.rightSidePWM = rightChannelPid_.getControll();
    // std::cout << "\x1B[2J\x1B[H";
    // std::cout << "cmd.leftSidePWM: " << cmd.leftSidePWM << std::endl;
    // std::cout << "cmd.rightSidePWM: " << cmd.rightSidePWM << std::endl;
    // std::cout << "leftVelocityCommand: " << leftVelocityCommand << std::endl;
    // std::cout << "rightVelocityCommand: " << rightVelocityCommand << std::endl;
    if (cmd.leftSidePWM != 0.0)
    {
      cmd.leftDirection = (leftChannelPid_.getPoint() > 0) ? 1 : 2;
      cmd.leftSidePWM = abs(cmd.leftSidePWM);
    }
    else
    {
      cmd.leftDirection = 0;
    }

    if (cmd.rightSidePWM != 0.0)
    {
      cmd.rightDirection = (rightChannelPid_.getPoint() > 0) ? 1 : 2;
      cmd.rightSidePWM = abs(cmd.rightSidePWM);
    }
    else
    {
      cmd.rightDirection = 0;
    }
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

  std::vector<hardware_interface::JointHandle> MotorController::getJoints()
  {
    std::vector<hardware_interface::JointHandle> output;
    output.push_back(frontLeftJointHandle);
    output.push_back(rearLeftJointHandle);
    output.push_back(frontRightJointHandle);
    output.push_back(rearRightJointHandle);

    return output;
  }
} // namespace vc200_driver
