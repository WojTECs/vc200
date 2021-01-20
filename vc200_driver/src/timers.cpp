#include "vc200_driver/timers.h"

namespace vc200_driver {
Timers::Timers(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
  : timerConfig(new Interface::DownstreamData::TimerConfigurationFrame)
  , serviceTimerUpstream(new Interface::UpstreamData::ServiceTimeFrame)
  , timersUpstream(new Interface::UpstreamData::TimersFrame)
  , timeSyncUpstream(new Interface::UpstreamData::TimeSyncFrame)
  , stClient_(st_if)
  , nh_(nh) {
  stClient_->addExpectedDataType(timeSyncUpstream);
  stClient_->addExpectedDataType(timersUpstream);
  stClient_->addExpectedDataType(serviceTimerUpstream);
}

}  // namespace vc200_driver