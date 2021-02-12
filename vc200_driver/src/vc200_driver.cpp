#ifndef vc_200_driver
#define vc_200_driver

#include "vc200_driver/vc200_driver.h"

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>

#include <vector>

namespace vc200_driver {

VC200Driver::VC200Driver() : connected_(false), running_(false) {
  try {
    stClientPtr_ = std::make_shared<STInterface::STInterfaceClientUDP>(1115, "172.17.10.2", "7");
    connected_ = true;
  } catch (const boost::exception &e) {
    std::string diag = diagnostic_information(e);
    ROS_FATAL("Exception received during STInterface creation: %s", diag.c_str());
    connected_ = false;
  }
}
VC200Driver::~VC200Driver() { stop(); }
bool VC200Driver::init(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) {
  // IMU init
  if (!connected_) {
    return false;
  }
  try {
    imuSensorsPtr_.reset(new IMU(stClientPtr_, priv_nh));
    // MotorController init
    motorControllerPtr_.reset(new MotorController(stClientPtr_, priv_nh));
    // Statistics init
    statsPtr_.reset(new Statistics(stClientPtr_, priv_nh));
    // Configurator init
    confPtr_.reset(new Configurator(stClientPtr_, priv_nh));
    // Timers init
    timersPtr_.reset(new Timers(stClientPtr_, priv_nh));

    //   imuSensorsPtr_->clibrate()
    laserRulerPtr_.reset(new LaserRuler(stClientPtr_,priv_nh));
  } catch (std::exception &e) {
    ROS_ERROR_STREAM("Exception during creating interfaces: \n" << e.what());
    return false;
  }
  return true;
}
void VC200Driver::readData() {
  imuSensorsPtr_->readData();
  motorControllerPtr_->readData();
  laserRulerPtr_->readData();
}
void VC200Driver::writeData() {
  imuSensorsPtr_->writeData();
  motorControllerPtr_->writeData();
}

std::vector<hardware_interface::JointHandle> VC200Driver::getPositionJoints() {
  std::vector<hardware_interface::JointHandle> posJoints;
  posJoints.clear();
  return posJoints;
}
std::vector<hardware_interface::JointHandle> VC200Driver::getVelocityJoints() {
  std::vector<hardware_interface::JointHandle> velJoints;
  velJoints.clear();
  for (auto &joint : motorControllerPtr_->getJoints()) {
    velJoints.push_back(joint);
  }
  return velJoints;
}
std::vector<hardware_interface::JointHandle> VC200Driver::getEffortJoints() {
  std::vector<hardware_interface::JointHandle> effortJoints;
  effortJoints.clear();
  return effortJoints;
}
std::vector<hardware_interface::ImuSensorHandle> VC200Driver::getImuJoints() {
  std::vector<hardware_interface::ImuSensorHandle> imuJoints;
  imuJoints.clear();
  imuJoints.push_back(imuSensorsPtr_->getHandle());
  return imuJoints;
}
void VC200Driver::updater() { stClientPtr_->run(); }
void VC200Driver::stop() {
  stClientPtr_->stop();
  if (running_) {
    updaterThread_.join();
    running_ = false;
  }
}
void VC200Driver::run() {
  if (!running_) {
    updaterThread_ = std::thread(&VC200Driver::updater, this);
    running_ = true;
  }
}
}  // namespace vc200_driver

#endif
