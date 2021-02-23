#include "vc200_driver/imu.h"
#define cov_matrix(x, y, z) \
  { x, 0, 0, 0, y, 0, 0, 0, z }
namespace vc200_driver {
IMU::IMU(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
    : angularVelocityCov cov_matrix(0.0001, 0.0001, 0.0001)
    , linearAccelerationCov cov_matrix(0.0001, 0.0001, 0.0001)
    , orientationCov cov_matrix(-1.0, -1.0, -1.0)
    , gyroXoffset(0.0)
    , gyroYoffset(0.0)
    , gyroZoffset(0.0)
    , accXoffset(0.0)
    , accYoffset(0.0)
    , accZoffset(0.230514)
    , stClient_(st_if)
    , nh_(nh) {
  // Accelerometer
  stClient_->addExpectedDataType(accelerometer.upstream);
  // Gyroscope
  stClient_->addExpectedDataType(gyroscope.upstream);
  // Magnetometer
  stClient_->addExpectedDataType(magnetometer.upstream);

  priv_nh_ = ros::NodeHandle(nh_, "imu");
  // name & frame id
  std::string frame_id = "stm";
  if (!priv_nh_.getParam("frame_id", frame_id)) {
    ROS_WARN_STREAM("Can not find frame id name of imu, default: " << frame_id);
  }
  imuData.frame_id = frame_id;

  std::string name = "stm_imu";
  if (!priv_nh_.getParam("name", name)) {
    ROS_WARN_STREAM("Can not find name of imu, default: " << name);
  }
  imuData.name = name;

  // covarinace
  std::vector<double> acceleration_covariance;
  acceleration_covariance.resize(9);
  std::vector<double> velocity_covariance;
  velocity_covariance.resize(9);
  std::vector<double> orientation_covariance;
  orientation_covariance.resize(9);

  if (!priv_nh_.getParam("acceleration_covariance", acceleration_covariance)) {
    ROS_WARN_STREAM("Can not find acceleration covariance matrix... setting default");
  } else {
    std::copy(acceleration_covariance.begin(), acceleration_covariance.end(), linearAccelerationCov);
  }
  imuData.linear_acceleration = linearAcceleration_;
  imuData.linear_acceleration_covariance = linearAccelerationCov;

  if (!priv_nh_.getParam("velocity_covariance", velocity_covariance)) {
    ROS_WARN_STREAM("Can not find velocity covariance matrix... setting default");
  } else {
    std::copy(velocity_covariance.begin(), velocity_covariance.end(), angularVelocityCov);
  }
  imuData.angular_velocity = angularVelocity_;
  imuData.angular_velocity_covariance = angularVelocityCov;

  if (!priv_nh_.getParam("orientation_covariance", orientation_covariance)) {
    ROS_WARN_STREAM("Can not find orientation covariance matrix... setting default");
  } else {
    std::copy(orientation_covariance.begin(), orientation_covariance.end(), orientationCov);
  }
  imuData.orientation = orientation_;
  imuData.orientation_covariance = orientationCov;

  // offsets

  std::vector<double> gyro_offset;
  gyro_offset.resize(3);
  if (!priv_nh_.getParam("gyro_offset", gyro_offset)) {
    ROS_WARN_STREAM("Can not find gyroscope offsets... setting default");
  } else {
    gyroXoffset = gyro_offset[0];
    gyroYoffset = gyro_offset[1];
    gyroZoffset = gyro_offset[2];
  }

  std::vector<double> acc_offset;
  acc_offset.resize(3);
  if (!priv_nh_.getParam("acc_offset", acc_offset)) {
    ROS_WARN_STREAM("Can not find accelerometer offsets... setting default");
  } else {
    accXoffset = acc_offset[0];
    accYoffset = acc_offset[1];
    accZoffset = acc_offset[2];
  }

  std::vector<double> mag_offset;
  mag_offset.resize(3);
  if (!priv_nh_.getParam("mag_offset", mag_offset)) {
    ROS_WARN_STREAM("Can not find magnetometer offsets... setting default");
  } else {
    magnetometer.upstream->offset_x = mag_offset[0];
    magnetometer.upstream->offset_y = mag_offset[1];
    magnetometer.upstream->offset_z = mag_offset[2];
  }

  std::vector<double> mag_scale;
  mag_scale.resize(3);
  if (!priv_nh_.getParam("mag_scale", mag_scale)) {
    ROS_WARN_STREAM("Can not find magnetometer scales... setting default");
  } else {
    magnetometer.upstream->scale_x = mag_scale[0];
    magnetometer.upstream->scale_y = mag_scale[1];
    magnetometer.upstream->scale_z = mag_scale[2];
  }

  // seting up imu handle
  imuSensorHandle = hardware_interface::ImuSensorHandle(imuData);
  angularVelocity_[0] = 0;
  angularVelocity_[1] = 0;
  angularVelocity_[2] = 0;
  linearAcceleration_[0] = 0;
  linearAcceleration_[1] = 0;
  linearAcceleration_[2] = 0;
}

void IMU::clibrate() {
  std::cout << "##### !!! WARNING !!! #####\n";
  std::cout << "  Gyro calibration proces  \n";
  std::cout << "    please wait for 30s    \n";
  std::cout << "   Do not move platform!   \n";
  std::cout << " Proces will start after 1s\n";
  std::cout << "##### !!! WARNING !!! #####\n";
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  int i = 0;

  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 15.0) {
    std::cout << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::cout << "\r";

    gyroscope.upstream->readData(gyroBuff);
    accelerometer.upstream->readData(accBuff);

    gyroXoffset = gyroXoffset + (1.0 / (i + 1.0) * (gyroBuff.xAxis - gyroXoffset));
    gyroYoffset = gyroYoffset + (1.0 / (i + 1.0) * (gyroBuff.yAxis - gyroYoffset));
    gyroZoffset = gyroZoffset + (1.0 / (i + 1.0) * (gyroBuff.zAxis - gyroZoffset));

    accXoffset = accXoffset + (1.0 / (i + 1.0) * (accBuff.xAxis - accXoffset));
    accYoffset = accYoffset + (1.0 / (i + 1.0) * (accBuff.yAxis - accYoffset));
    accZoffset = accZoffset + (1.0 / (i + 1.0) * (accBuff.zAxis - accZoffset));
    i++;
  }
  accZoffset = accZoffset - 9.80665;

  std::cout << std::setw(18) << std::left << ("GYRO [x]: " + std::to_string(gyroXoffset) + " ");
  std::cout << std::setw(18) << std::left << ("[y]: " + std::to_string(gyroYoffset) + " ");
  std::cout << std::setw(18) << std::left << ("[z]: " + std::to_string(gyroZoffset) + " ") << "\n";
  std::cout << std::setw(18) << std::left << ("ACC [x]: " + std::to_string(accXoffset) + " ");
  std::cout << std::setw(18) << std::left << ("[y]: " + std::to_string(accYoffset) + " ");
  std::cout << std::setw(18) << std::left << ("[z]: " + std::to_string(accZoffset) + " ") << "\n";
}
void IMU::readData() {
  accelerometer.upstream->readData(accBuff);
  gyroscope.upstream->readData(gyroBuff);
  // magnetometer.upstream->readData(magBuff);

  accBuff.xAxis -= accXoffset;
  accBuff.yAxis -= accYoffset;
  accBuff.zAxis -= accZoffset;

  gyroBuff.xAxis -= gyroXoffset;
  gyroBuff.yAxis -= gyroYoffset;
  gyroBuff.zAxis -= gyroZoffset;

  linearAcceleration_[0] = accBuff.xAxis;
  linearAcceleration_[1] = accBuff.yAxis;
  linearAcceleration_[2] = accBuff.zAxis;

  angularVelocity_[0] = gyroBuff.xAxis;
  angularVelocity_[1] = gyroBuff.yAxis;
  angularVelocity_[2] = gyroBuff.zAxis;
}
void IMU::writeData() {}

hardware_interface::ImuSensorHandle IMU::getHandle() const { return imuSensorHandle; }
}  // namespace vc200_driver