#include "stalker_driver/laser_ruler.h"

namespace Interface {
namespace UpstreamData {
LaserRulerFrame::LaserRulerFrame() {
  protocolIndentificator = uint8_t{0x12};
  datasetBinarySize = 0x8;
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

  for (int i = 0; i < 8; i++){
    handleSavingData(data.scan[i], iDataStream[i]);
  }
}

void LaserRulerFrame::doTheProcessing() {}

void LaserRulerFrame::handleSavingData(float &data, uint8_t iDataStream) {
  if (iDataStream == 255) {
    data = std::numeric_limits<float>::infinity();
  } else {
    data = (float)iDataStream / 1000.0;
  }
}

}  // namespace UpstreamData
}  // namespace Interface
