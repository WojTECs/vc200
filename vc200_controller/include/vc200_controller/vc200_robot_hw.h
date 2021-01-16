#ifndef VC200_ROBOT_HW_H
#define VC200_ROBOT_HW_H

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "vc200_driver/vc200_driver.h"

namespace vc200_robot_hw {
class VC200RobotHW : public hardware_interface::RobotHW {
 public:
  VC200RobotHW();
  ~VC200RobotHW();

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

 private:
  double rateHz_;

  std::unique_ptr<vc200_driver::VC200Driver> driverPtr_;

  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::PositionJointInterface positionJointInterface_;
  hardware_interface::VelocityJointInterface velocityJointInterface_;
  hardware_interface::EffortJointInterface effortJointInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
};

}  // namespace vc200_robot_hw

#endif