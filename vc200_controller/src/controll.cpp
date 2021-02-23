#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <iostream>

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/STInterfaceClientUDP.h"
#include "stalker_driver/UpstreamDataType.h"
#include "vc200_controller/vc200_robot_hw.h"
#include "vc200_driver/component_types.h"

using namespace std;
int main(int argc, char** argv) {
  ros::init(argc, argv, "ll_robot_hw_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");


  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  int spinners = 2;
  // priv_nh.param("spinners", spinners, 2);

  ros::AsyncSpinner spinner(spinners);
  spinner.start();

  vc200_robot_hw::VC200RobotHW robot_hw;
  int num_retries = 5;

  if (robot_hw.init(nh, priv_nh) == false) {
    while (num_retries && ros::ok()) {
      ros::Duration(3.0).sleep();

      if (robot_hw.init(nh, priv_nh)) {
        break;
      } else {
        num_retries--;
      }
    }
    ROS_ERROR_STREAM("Can not initialize node. Exiting...");
    //    ros::shutdown();
    //    ros::waitForShutdown();
    return 0;
  }
  controller_manager::ControllerManager controller_mngr(&robot_hw);
  ros::Time prevTime = ros::Time::now();
  double rate_hz = 100;
  ros::Rate rate(rate_hz);

  while (ros::ok()) {
    ros::Time currentTime = ros::Time::now();
    ros::Duration period = ros::Duration(currentTime - prevTime);
    prevTime = currentTime;
    // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "Loop", "Main Loop duration is " <<
    // period << " sec");
    // cout << " ok" << endl;
    robot_hw.read(currentTime, period);

    if ((period.toSec() > (2.0 * 1.0 / rate_hz)) == true) {
      controller_mngr.update(currentTime, period,
                             true);  // Reset controllers if error
      // ROS_WARN_STREAM_THROTTLE(0.1,
      //                          "Reseting controllers! Main Loop duration was
      //                          " << period.toSec() << "s should be " << 1.0 /
      //                          rate_hz
      //                                                                          << "s");
    } else {
      controller_mngr.update(currentTime, period, false);
    }

    robot_hw.write(currentTime, period);
    rate.sleep();
  }

  ros::waitForShutdown();
}