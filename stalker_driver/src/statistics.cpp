#include "stalker_driver/statistics.h"

namespace Interface {
namespace UpstreamData {
PackageStatisticsFrame::PackageStatisticsFrame() {
  protocolIndentificator = uint8_t{ 0xFE };
  datasetBinarySize = 8;
}

PackageStatisticsFrame::~PackageStatisticsFrame() {
}

void PackageStatisticsFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  if (iDataSize != datasetBinarySize) {
    std::cout << "Bad Time Sync frame received. Length is mismatching" << std::endl;
    return;
  }
  std::lock_guard<std::mutex> lock(dataMutex);
  data.sentPackages = (iDataStream[0] << 24) | (iDataStream[1] << 16) | (iDataStream[2] << 8) | (iDataStream[3] & 0xFF);
  data.malformedPackages =
      (iDataStream[4] << 24) | (iDataStream[5] << 16) | (iDataStream[6] << 8) | (iDataStream[7] & 0xFF);
  doTheProcessing();
}
void PackageStatisticsFrame::readData(StatisticsDataset& dest) {
  std::lock_guard<std::mutex> lock(dataMutex);
  dest = data;
}
void PackageStatisticsFrame::doTheProcessing() {
}

// std::unique_ptr<Interface::UpstreamDataType> PackageStatisticsFrame::getClone() {
//   std::unique_ptr<PackageStatisticsFrame> packageStatisticsFrame(new PackageStatisticsFrame);

//   packageStatisticsFrame->protocolIndentificator = protocolIndentificator;
//   packageStatisticsFrame->datasetBinarySize = datasetBinarySize;

//   return std::move(packageStatisticsFrame);
// }
}  // namespace UpstreamData
}  // namespace Interface