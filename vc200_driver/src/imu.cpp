#include "vc200_driver/imu.h"
namespace vc200_driver
{
IMU::IMU(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, std::string n)
  : Component(st_if, n)
  // , angularVelocityCov{ 0.04940425075, 0, 0, 0, 0.03813046707, 0, 0, 0, 0.09872148499 }
  // , linearAccelerationCov{ 0.01420020585, 0, 0, 0, 0.01024185714, 0, 0, 0, 0.009558967642 }
  // , orientationCov{ 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001 }
  // , orientationCov{ -1, 0, 0, 0, -1, 0, 0, 0, -1 }
  , angularVelocityCov{ 0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001 }
  , linearAccelerationCov{ 0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001 }
  , orientationCov{ -1, 0, 0, 0, -1, 0, 0, 0, -1 }
  , gyroXoffset(0.0)
  , gyroYoffset(0.0)
  , gyroZoffset(0.0)
  , accXoffset(0.0)
  , accYoffset(0.0)
  , accZoffset(0.230514)
{
  // Accelerometer
  stClient_->addExpectedDataType(accelerometer.upstream);
  // Gyroscope
  stClient_->addExpectedDataType(gyroscope.upstream);
  // Magnetometer
  stClient_->addExpectedDataType(magnetometer.upstream);
  // imuSensorHandle.
  imuData.name = "stm_imu";
  imuData.frame_id = "stm";
  imuData.angular_velocity = angularVelocity_;
  imuData.angular_velocity_covariance = angularVelocityCov;

  imuData.linear_acceleration = linearAcceleration_;
  imuData.linear_acceleration_covariance = linearAccelerationCov;

  imuData.orientation = orientation_;
  imuData.orientation_covariance = orientationCov;

  imuSensorHandle = hardware_interface::ImuSensorHandle(imuData);
  angularVelocity_[0] = 0;
  angularVelocity_[1] = 0;
  angularVelocity_[2] = 0;
  linearAcceleration_[0] = 0;
  linearAcceleration_[1] = 0;
  linearAcceleration_[2] = 0;
}

void IMU::clibrate()
{
  std::cout << "##### !!! WARNING !!! #####\n";
  std::cout << "  Gyro calibration proces  \n";
  std::cout << "    please wait for 30s    \n";
  std::cout << "   Do not move platform!   \n";
  std::cout << " Proces will start after 1s\n";
  std::cout << "##### !!! WARNING !!! #####\n";
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  int i = 0;

  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 15.0)
  {
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
void IMU::readData()
{
  accelerometer.upstream->readData(accBuff);
  gyroscope.upstream->readData(gyroBuff);
  magnetometer.upstream->readData(magBuff);

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
void IMU::writeData()
{
}

}  // namespace vc200_driver