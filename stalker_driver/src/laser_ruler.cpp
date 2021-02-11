#include "stalker_driver/laser_ruler.h"

namespace Interface {
namespace UpstreamData {
LaserRulerFrame::LaserRulerFrame() {
  protocolIndentificator = uint8_t{0x12};
  datasetBinarySize = 0x4;
}
void LaserRulerFrame::readData(LaserRulerDataset& dest) {
  std::lock_guard<std::mutex> lock(dataMutex);
  dest = data;
}
LaserRulerFrame::~LaserRulerFrame() {}
void LaserRulerFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  if (iDataSize % datasetBinarySize != 0) {
    std::cout << "Bad laser ruler frame received. Lenght is mismatched" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(dataMutex);
  if (iDataStream[0] == 255) {
    data.scan[0] = std::numeric_limits<float>::infinity();
  } else {
    data.scan[0] = iDataStream[0] / 1000.0;
  }
  if (iDataStream[1] == 255) {
    data.scan[1] = std::numeric_limits<float>::infinity();
  } else {
    data.scan[1] = iDataStream[1] / 1000.0;
  }
  if (iDataStream[2] == 255) {
    data.scan[2] = std::numeric_limits<float>::infinity();
  } else {
    data.scan[2] = iDataStream[2] / 1000.0;
  }
  if (iDataStream[3] == 255) {
    data.scan[3] = std::numeric_limits<float>::infinity();
  } else {
    data.scan[3] = iDataStream[3] / 1000.0;
  }
  // doTheProcessing();
}

void LaserRulerFrame::doTheProcessing() {
}

}  // namespace UpstreamData
}  // namespace Interface