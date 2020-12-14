#include "vc200_driver/timers.h"

namespace vc200_driver
{
Timers::Timers(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, std::string _name)
  : timerConfig(new Interface::DownstreamData::TimerConfigurationFrame)
  , serviceTimerUpstream(new Interface::UpstreamData::ServiceTimeFrame)
  , timersUpstream(new Interface::UpstreamData::TimersFrame)
  , timeSyncUpstream(new Interface::UpstreamData::TimeSyncFrame)
  , Component(st_if, _name)
{
  stClient_->addExpectedDataType(timeSyncUpstream);
  stClient_->addExpectedDataType(timersUpstream);
  stClient_->addExpectedDataType(serviceTimerUpstream);
}

}  // namespace vc200_driver