#pragma once

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>

#include <vector>

#include "stalker_driver/STInterfaceClientUDP.h"

namespace vc_200_driver {
class VC200Driver {
 public:
  bool init();
  std::vector<hardware_interface::JointHandle> getPositionJoints();
  std::vector<hardware_interface::JointHandle> getVelocityJoints();
  std::vector<hardware_interface::JointHandle> getEffortJoints();
  std::vector<hardware_interface::JointHandle> getImuJoints();
  void readData();
  void writeData();

 private:
  // komunikacja z stm32 i obikety przechowujace informacje
  // uchwyty do danych lub interfejsy do danych
  // publishery i diagnostyka
  // init
  // jointy // poszczegolnych elementow
};
}  // namespace vc_200_driver