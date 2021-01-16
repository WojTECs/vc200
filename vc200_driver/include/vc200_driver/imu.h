#pragma once

#include <hardware_interface/imu_sensor_interface.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <thread>

#include "stalker_driver/STInterfaceClientUDP.h"
#include "vc200_driver/accelerometer.h"
#include "vc200_driver/gyroscope.h"
#include "vc200_driver/magnetometer.h"

namespace vc200_driver {
class IMU {
 private:
  /* data */

  double angularVelocity_[3];
  double angularVelocityCov[9];
  double linearAcceleration_[3];
  double linearAccelerationCov[9];
  double orientation_[4];
  double orientationCov[9];

  Interface::UpstreamData::AccelerometerDataset accBuff;
  Interface::UpstreamData::GyroscopeDataset gyroBuff;
  Interface::UpstreamData::MagnetometerDataset magBuff;

  double gyroXoffset;
  double gyroYoffset;
  double gyroZoffset;
  double accXoffset;
  double accYoffset;
  double accZoffset;
  hardware_interface::ImuSensorHandle::Data imuData;
  hardware_interface::ImuSensorHandle imuSensorHandle;
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

 public:
  IMU(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh);
  std::vector<float> getRPY();
  Accelerometer accelerometer;
  Gyroscope gyroscope;
  Magnetometer magnetometer;

  hardware_interface::ImuSensorHandle getHandle() const;
  void readData();
  void writeData();
  void clibrate();
};

}  // namespace vc200_driver