#include "vc200_driver/component.h"
#include "stalker_driver/timers.h"
#include <string>
namespace vc200_driver {
class Timers : public Component {
public:
  Timers(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, std::string _name);

private:
  std::shared_ptr<Interface::DownstreamData::TimerConfigurationFrame> timerConfig;
  std::shared_ptr<Interface::UpstreamData::ServiceTimeFrame> serviceTimerUpstream;
  std::shared_ptr<Interface::UpstreamData::TimersFrame> timersUpstream;
  std::shared_ptr<Interface::UpstreamData::TimeSyncFrame> timeSyncUpstream;
};
}  // namespace vc200_driver
