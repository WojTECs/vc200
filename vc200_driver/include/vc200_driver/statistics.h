#include "vc200_driver/component.h"
#include "stalker_driver/statistics.h"
#include <string>

namespace vc200_driver {
class Statistics : public Component {
public:
  Statistics(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, std::string _name);
  std::shared_ptr<Interface::UpstreamData::PackageStatisticsFrame> upstream;
};
}  // namespace vc200_driver
