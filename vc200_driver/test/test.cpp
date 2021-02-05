#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <memory>

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/UpstreamDataType.h"
#include "vc200_driver/component_types.h"
#include "vc200_driver/vc200_driver.h"
#define RAD_TO_DEG 180.0 / M_PI
// void clibrateGyro();
using namespace std;
float speedCMD = 0;
void speedCb(const std_msgs::Float32ConstPtr msg) { speedCMD = msg->data; }

int main(int argc, char** argv) {
  ros::init(argc, argv, "Stalker");
  
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  vc200_driver::VC200Driver driver;

  if (!driver.init(nh, priv_nh)) {
    return 0;
  }
  
  driver.run();

  ros::Rate r(10);

  while (ros::ok()) {
    driver.readData();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}