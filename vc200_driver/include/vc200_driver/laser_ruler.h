#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserEcho.h>

#include <future>
#include <memory>
#include <string>

#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/laser_ruler.h"
namespace vc200_driver {
class LaserRuler {
 public:
  LaserRuler(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh);
  std::shared_ptr<Interface::UpstreamData::LaserRulerFrame> laserRulerUpstream;
  std::shared_ptr<STInterface::STInterfaceClientUDP> stClient_;
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  void readData();
  void writeData();
  void publish();

 private:
  Interface::UpstreamData::LaserRulerDataset data;
  int numberOfSensors;
  int maxDistance;
  int minDistance;
  int rulerLenght;
  int distanceIncremet;
  std::vector<float> scans;
  sensor_msgs::LaserEcho msg;

  ros::Publisher distPublisher;
  std::future<void> future_task_;
};
}  // namespace vc200_driver