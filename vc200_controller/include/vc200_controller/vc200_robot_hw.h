#ifndef VC200_ROBOT_HW_H
#define VC200_ROBOT_HW_H

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <vc200_driver/vc_200_driver.h>

namespace vc200_robot_hw
{
class VC200RobotHw : public hardware_interface::RobotHW
{
public:
  VC200RobotHw();
  ~VC200RobotHw();
  bool init();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  double rateHz_;

  vc_200_driver::VC200Driver driver_;

  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::VelocityJointInterface velocityJointInterface_;
  hardware_interface::EffortJointInterface effortJointInterface_;
  hardware_interface::PositionJointInterface positionJointInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
};

}  // namespace vc_200_robot_hw

#endif