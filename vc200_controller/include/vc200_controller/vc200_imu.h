#ifndef VC200_ROBOT_HW_H
#define VC200_ROBOT_HW_H

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
// #include "vc200_driver/vc_200_driver.h"
#include "vc200_driver/imu.h"

namespace vc200_robot_hw {
class VC200RobotHw : public hardware_interface::RobotHW {
 public:
  VC200RobotHw(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if,
               ros::NodeHandle& nh);
  ~VC200RobotHw(){};
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

 private:

  double rateHz_;
  vc200_driver::IMU imu_ctrl;
  ros::ServiceServer imuCalibration;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
};

}  // namespace vc200_robot_hw

#endif