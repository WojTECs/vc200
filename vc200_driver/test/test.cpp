#include <memory>
#include <ros/ros.h>
#include "stalker_driver/UpstreamDataType.h"
#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/STInterfaceClientUDP.h"
#include "vc200_driver/component_types.h"

#define RAD_TO_DEG 180.0 / M_PI
// void clibrateGyro();

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "Stalker");
  std::shared_ptr<STInterface::STInterfaceClientUDP> stCli;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  try
  {
    stCli = std::make_shared<STInterface::STInterfaceClientUDP>(1115, "192.168.1.10", "7");
  }
  catch (const boost::exception& e)
  {
    std::string diag = diagnostic_information(e);
    ROS_FATAL("Exception received during STInterface creation: %s", diag.c_str());
    return 0;
  }
  Interface::UpstreamData::AccelerometerDataset accData;
  Interface::UpstreamData::AccelerometerDataset accData2;

  Interface::UpstreamData::GyroscopeDataset gyroData;
  Interface::UpstreamData::MagnetometerDataset magData;

  vc200_driver::IMU imu(stCli, "imu");
  vc200_driver::Configurator conf(stCli, "configurator");
  vc200_driver::MotorController motor(stCli, "motor control");
  vc200_driver::Statistics stats(stCli, "statistics");
  vc200_driver::Timers timers(stCli, "timers");

  // stCli->run();
  std::thread updater([&]() { stCli->run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  while (1)
  {
    // imu.readData();
    imu.accelerometer.upstream->readData(accData);

    std::cout << std::setw(18) << std::left << ("[x1]: " + std::to_string(accData.xAxis) + " ");
    std::cout << std::setw(18) << std::left << ("[y1]: " + std::to_string(accData.yAxis) + " ");
    std::cout << std::setw(18) << std::left << ("[z1]: " + std::to_string(accData.zAxis) + " ") << "\n";
    std::cout << "\x1B[2J\x1B[H";
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  stCli->stop();
  updater.join();
  return 0;
}