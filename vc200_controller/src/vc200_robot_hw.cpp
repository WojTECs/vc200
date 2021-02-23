#include "vc200_controller/vc200_robot_hw.h"
namespace vc200_robot_hw {
VC200RobotHW::VC200RobotHW() { driverPtr_.reset(); }
VC200RobotHW::~VC200RobotHW() {
  driverPtr_->stop();
  driverPtr_.reset();
}

bool VC200RobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  driverPtr_.reset(new vc200_driver::VC200Driver());
  if (!driverPtr_->init(root_nh, robot_hw_nh)) {
    return false;
  }

  // position joints
  for (auto& joint : driverPtr_->getPositionJoints()) {
    jointStateInterface_.registerHandle(joint);
    positionJointInterface_.registerHandle(joint);
  }

  for (auto& joint : driverPtr_->getVelocityJoints()) {
    jointStateInterface_.registerHandle(joint);
    velocityJointInterface_.registerHandle(joint);
  }

  for (auto& joint : driverPtr_->getEffortJoints()) {
    jointStateInterface_.registerHandle(joint);
    effortJointInterface_.registerHandle(joint);
  }

  for (auto& joint : driverPtr_->getImuJoints()) {
    imuSensorInterface_.registerHandle(joint);
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&positionJointInterface_);
  registerInterface(&velocityJointInterface_);
  registerInterface(&effortJointInterface_);
  registerInterface(&imuSensorInterface_);

  driverPtr_->run();
  return true;
}

void VC200RobotHW::read(const ros::Time& time, const ros::Duration& period) {
  if (driverPtr_) {
    driverPtr_->readData();
  }
}
void VC200RobotHW::write(const ros::Time& time, const ros::Duration& period) {
  if (driverPtr_) {
    driverPtr_->writeData();
  }
}
}  // namespace vc200_robot_hw