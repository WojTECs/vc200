#include "vc200_driver/statistics.h"

namespace vc200_driver {
Statistics::Statistics(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, std::string _name)
  : upstream(new Interface::UpstreamData::PackageStatisticsFrame), Component(st_if, _name) {
  stClient_->addExpectedDataType(upstream);
}
}  // namespace vc200_driver