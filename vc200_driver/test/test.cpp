#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <memory>

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/UpstreamDataType.h"
#include "vc200_driver/component_types.h"

#define RAD_TO_DEG 180.0 / M_PI
// void clibrateGyro();
using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "Stalker");
  ros::NodeHandle nh("~");

  // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  // ros::console::levels::Debug);
  std::shared_ptr<STInterface::STInterfaceClientUDP> stCli;

  try {
    stCli = std::make_shared<STInterface::STInterfaceClientUDP>(1115, "192.168.1.10", "7");
  } catch (const boost::exception& e) {
    std::string diag = diagnostic_information(e);
    ROS_FATAL("Exception received during STInterface creation: %s", diag.c_str());
    return 0;
  }

  Interface::UpstreamData::AccelerometerDataset accData;
  Interface::UpstreamData::GyroscopeDataset gyroData;
  Interface::UpstreamData::MagnetometerDataset magData;

  vc200_driver::IMU imu(stCli, nh);
  vc200_driver::Configurator conf(stCli, nh);
  vc200_driver::MotorController motor(stCli, nh);
  vc200_driver::Statistics stats(stCli, nh);
  vc200_driver::Timers timers(stCli, nh);

  // stCli->run();
  std::thread updater([&]() { stCli->run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  Interface::UpstreamData::EncoderDataset enc;

  std_msgs::Float32 leftSpeedMsg;
  std_msgs::Float32 leftDistMsg;
  std_msgs::Float32 rightSpeedMsg;
  std_msgs::Float32 rightDistMsg;

  ros::Publisher leftSpeedPub = nh.advertise<std_msgs::Float32>("leftSpeedPub", 1);
  ros::Publisher leftDistPub = nh.advertise<std_msgs::Float32>("leftDistPub", 1);
  ros::Publisher rightSpeedPub = nh.advertise<std_msgs::Float32>("rightSpeedPub", 1);
  ros::Publisher rightDistPub = nh.advertise<std_msgs::Float32>("rightDistPub", 1);
  ros::Rate r(10);
  Interface::DownstreamData::MovementCommandDataset cmd;
  std::once_flag f;
  while (ros::ok()) {
    // imu.readData();
    // imu.accelerometer.upstream->readData(accData);
    // imu.gyroscope.upstream->readData(gyroData);
    // imu.magnetometer.upstream->readData(magData);
    // std::cout << "ACC";
    // std::cout << std::setw(18) << std::left << ("[x]: " +
    // std::to_string(accData.xAxis) + " "); std::cout << std::setw(18) <<
    // std::left << ("[y]: " + std::to_string(accData.yAxis) + " "); std::cout
    // << std::setw(18) << std::left << ("[z]: " + std::to_string(accData.zAxis)
    // + " ") << "\n"; std::cout << "GYR"; std::cout << std::setw(18) <<
    // std::left << ("[x]: " + std::to_string(gyroData.xAxis) + " "); std::cout
    // << std::setw(18) << std::left << ("[y]: " +
    // std::to_string(gyroData.yAxis) + " "); std::cout << std::setw(18) <<
    // std::left << ("[z]: " + std::to_string(gyroData.zAxis) + " ") << "\n";
    // std::cout << "MAG";
    // std::cout << std::setw(18) << std::left << ("[x]: " +
    // std::to_string(magData.xAxis) + " "); std::cout << std::setw(18) <<
    // std::left << ("[y]: " + std::to_string(magData.yAxis) + " "); std::cout
    // << std::setw(18) << std::left << ("[z]: " + std::to_string(magData.zAxis)
    // + " ") << "\n";
    ros::spinOnce();
    r.sleep();
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // std::cout << "\x1B[2J\x1B[H";
  }
  stCli->stop();
  updater.join();
  return 0;
}