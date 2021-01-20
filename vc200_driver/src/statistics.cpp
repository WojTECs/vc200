#include "vc200_driver/statistics.h"

namespace vc200_driver {
Statistics::Statistics(std::shared_ptr<STInterface::STInterfaceClientUDP> st_if, ros::NodeHandle &nh)
  : upstream(new Interface::UpstreamData::PackageStatisticsFrame), stClient_(st_if), nh_(nh) {
  stClient_->addExpectedDataType(upstream);
}
}  // namespace vc200_driver