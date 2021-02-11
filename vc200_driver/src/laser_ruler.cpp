#include "vc200_driver/laser_ruler.h"

namespace vc200_driver {
LaserRuler::LaserRuler(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
    : laserRulerUpstream(new Interface::UpstreamData::LaserRulerFrame), stClient_(st_if), nh_(nh) {
  stClient_->addExpectedDataType(laserRulerUpstream);
  scans.clear();
  scans.resize(0);
  priv_nh_ = ros::NodeHandle(nh_, "laser_ruler");

  numberOfSensors = 4;
  if (!priv_nh_.getParam("number_of_sensors", numberOfSensors)) {
    ROS_WARN_STREAM("[Laser ruler]: Can not find number of sensors, default: " << numberOfSensors);
  }
  scans.resize(numberOfSensors);

  maxDistance = 0;
  if (!priv_nh_.getParam("max_distance", maxDistance)) {
    ROS_WARN_STREAM("[Laser ruler]: Can not find max distance, default: " << maxDistance);
  }

  minDistance = 0;
  if (!priv_nh_.getParam("min_distance", minDistance)) {
    ROS_WARN_STREAM("[Laser ruler]: Can not find min distance, default: " << minDistance);
  }

  rulerLenght = 0;
  if (!priv_nh_.getParam("ruler_lenght", rulerLenght)) {
    ROS_WARN_STREAM("[Laser ruler]: Can not find ruler lenght, default: " << rulerLenght);
  }

  distanceIncremet = 0;
  if (!priv_nh_.getParam("distance_incremet", distanceIncremet)) {
    ROS_WARN_STREAM("[Laser ruler]: Can not find distance incremet, default: " << distanceIncremet);
  }

  distPublisher = priv_nh_.advertise<sensor_msgs::LaserEcho>("scan", 1, true);
  future_task_ = std::async([]() {});
}

void LaserRuler::publish() {
  if ((future_task_.valid()) && (scans.size() > 0)) {
    future_task_ = std::async([this]() {
      // std::cout << "dupsko\n";
      msg.echoes = this->scans;
      this->distPublisher.publish(msg);
    });
  }
}

void LaserRuler::readData() {
  // std::cout << "se czytam z linijki"<<std::endl;
  laserRulerUpstream->readData(data);
  for (size_t i = 0; i < numberOfSensors; i++) {
    scans[i] = data.scan[i];
  }
  publish();
}

void LaserRuler::writeData() {  // nothing
}

}  // namespace vc200_driver