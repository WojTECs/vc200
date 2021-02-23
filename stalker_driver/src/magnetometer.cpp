#include "stalker_driver/magnetometer.h"

#include <boost/exception/diagnostic_information.hpp>

namespace Interface {
namespace DownstreamData {
MagnetometerFrame::MagnetometerFrame() {
  potocolIndentificator = "MagnetometerFrame";
  stIdentifier = 0x06;
}

MagnetometerFrame::~MagnetometerFrame() {}

std::vector<uint8_t> MagnetometerFrame::serialize() {
  std::vector<uint8_t> output(3);
  std::lock_guard<std::mutex> lock(dataMutex);
  output[0] = stIdentifier;
  output[1] = command.registryAddress;
  output[2] = command.registryValue;
  return output;
}

void MagnetometerFrame::setCommnad(MagnetometerCommandDataset& in) {
  std::lock_guard<std::mutex> lock(dataMutex);
  command = in;
  doTheProcessing();
}

void MagnetometerFrame::doTheProcessing() {}

}  // namespace DownstreamData

namespace UpstreamData {
MagnetometerFrame::MagnetometerFrame()
    : x(0.05, 0)
    , y(0.05, 0)
    , z(0.05, 0)
    // , offset_x(-644.25)
    , offset_x(-0.0)
    // , offset_y(236.25)
    , offset_y(0.0)
    // , offset_z(731.4)
    , offset_z(0.0)
    // , scale_x(1.0112138016019716)
    , scale_x(0.0)
    // , scale_y(1.0112138016019716)
    , scale_y(0.0)
    // , scale_z(0.9783023366714355)
    , scale_z(0.0) {
  protocolIndentificator = uint8_t{0x06};
  datasetBinarySize = 10;
}

void UpstreamData::MagnetometerFrame::filter() {
  data = {};
  for (auto const& dataset : datasets) {
    // data.xAxis += dataset.xAxis;
    x.filter(dataset.xAxis);
    // data.yAxis += dataset.yAxis;
    y.filter(dataset.yAxis);
    // data.zAxis += dataset.zAxis;
    z.filter(dataset.zAxis);
    // std::cout << dataset.xAxis << "," << dataset.yAxis << "," << dataset.zAxis << std::endl;
  }
  data.timestamp = datasets.back().timestamp;

  data.xAxis = (x.getOutput() - offset_x) * scale_x;
  data.yAxis = (y.getOutput() - offset_y) * scale_y;
  data.zAxis = (z.getOutput() - offset_z) * scale_z;

  // data.xAxis = ((data.xAxis / (float)datasets.size()) - offset_x) * scale_x;
  // data.yAxis = ((data.yAxis / (float)datasets.size()) - offset_y) * scale_y;
  // data.zAxis = ((data.zAxis / (float)datasets.size()) - offset_z) * scale_z;
}
void UpstreamData::MagnetometerFrame::readData(MagnetometerDataset& dest) {
  std::lock_guard<std::mutex> lock(dataMutex);
  dest = data;
}

MagnetometerFrame::~MagnetometerFrame() {}

void MagnetometerFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  if (iDataSize % datasetBinarySize != 0) {
    std::cout << "Bad Magnetometer frame received. Length is mismatched" << std::endl;
    return;
  }

  int dataCount = iDataSize / datasetBinarySize;
  std::lock_guard<std::mutex> lock(dataMutex);

  datasets = std::vector<MagnetometerDataset>(dataCount);

  int byteShift = 0;

  for (int i = 0; i < dataCount; i++) {
    byteShift = i * datasetBinarySize;
    datasets[i].xAxis = int16_t((iDataStream[0 + byteShift] << 8) | (iDataStream[1 + byteShift] & 0xFF));

    datasets[i].yAxis = int16_t((iDataStream[2 + byteShift] << 8) | (iDataStream[3 + byteShift] & 0xFF));

    datasets[i].zAxis = int16_t((iDataStream[4 + byteShift] << 8) | (iDataStream[5 + byteShift] & 0xFF));

    datasets[i].timestamp = (iDataStream[6 + byteShift] << 24) | (iDataStream[7 + byteShift] << 16) |
                            (iDataStream[8 + byteShift] << 8) | (iDataStream[9 + byteShift] & 0xFF);
  }
  doTheProcessing();
  filter();
}

void MagnetometerFrame::doTheProcessing() {
  for (int i = 0; i < datasets.size(); i++) {
    // tranforming on mG - miligauss using board xnucleo iks01a3
    // multipliers are derived from internal board settings for magnetometer with +-50G max range
    datasets[i].xAxis = datasets[i].xAxis * 1.5;
    datasets[i].yAxis = datasets[i].yAxis * 1.5;
    datasets[i].zAxis = datasets[i].zAxis * 1.5;
  }
}

// std::unique_ptr<UpstreamDataType> MagnetometerFrame::getClone()
// {
//   std::unique_ptr<MagnetometerFrame> magnetometerFrame(new MagnetometerFrame);

//   magnetometerFrame->protocolIndentificator = protocolIndentificator;
//   magnetometerFrame->datasetBinarySize = datasetBinarySize;
//   return std::move(magnetometerFrame);
// }
}  // namespace UpstreamData
}  // namespace Interface