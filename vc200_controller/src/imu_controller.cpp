#include "vc200_controller/vc200_imu.h"

namespace vc200_robot_hw {
VC200RobotHw::VC200RobotHw(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle& nh)
    : imu_ctrl(st_if, nh) {
  rateHz_ = 100;
}
bool VC200RobotHw::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  for (auto i : imuSensorInterface_.getNames()) {
    std::cout << i << std::endl;
  }
  imuSensorInterface_.registerHandle(imu_ctrl.getHandle());

  std::cout << imu_ctrl.getHandle().getName() << std::endl;

  registerInterface(&imuSensorInterface_);

  // for (auto i : imuSensorInterface_.getNames())
  // {
  //   std::cout << i << std::endl;
  // }
  // for (auto i : this->getNames())
  // {
  //   std::cout << i << std::endl;
  // }

  return true;
}
void VC200RobotHw::read(const ros::Time& time, const ros::Duration& period) { imu_ctrl.readData(); }

void VC200RobotHw::write(const ros::Time& time, const ros::Duration& period) { imu_ctrl.writeData(); }

}  // namespace vc200_robot_hw