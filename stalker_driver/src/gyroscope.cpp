#include "stalker_driver/gyroscope.h"

#include <math.h>

#include <boost/exception/diagnostic_information.hpp>
namespace Interface {
namespace DownstreamData {
GyroscopeFrame::GyroscopeFrame() {
  potocolIndentificator = "GyroscopeFrame";
  stIdentifier = 0x05;
}

GyroscopeFrame::~GyroscopeFrame() {}

std::vector<uint8_t> GyroscopeFrame::serialize() {
  std::vector<uint8_t> output(3);
  std::lock_guard<std::mutex> lock(dataMutex);
  output[0] = stIdentifier;
  output[1] = command.registryAddress;
  output[2] = command.registryValue;

  return output;
}

void GyroscopeFrame::setCommand(GyroscopeCommandDataset& in) {
  std::lock_guard<std::mutex> lock(dataMutex);
  command = in;
  doTheProcessing();
}

void GyroscopeFrame::doTheProcessing() {}
}  // namespace DownstreamData

namespace UpstreamData {
GyroscopeFrame::GyroscopeFrame() : x(0.09, 0), y(0.09, 0), z(0.09, 0) {
  protocolIndentificator = 0x05;
  datasetBinarySize = 10;
}

void UpstreamData::GyroscopeFrame::filter() {
  data = {};
  for (auto const& dataset : datasets) {
    // data.xAxis += dataset.xAxis;
    x.filter(dataset.xAxis);
    // data.yAxis += dataset.yAxis;
    y.filter(dataset.yAxis);
    // data.zAxis += dataset.zAxis;
    z.filter(dataset.zAxis);
    // std::cout << std::setw(18) << std::left << ("[x]: " + std::to_string(data.xAxis) + " ");
    // std::cout << std::setw(18) << std::left << ("[y]: " + std::to_string(data.yAxis) + " ");
    // std::cout << std::setw(18) << std::left << ("[z]: " + std::to_string(data.zAxis) + " ") << "\n";
    // std::cout << "\x1B[2J\x1B[H";
  }
  data.timestamp = datasets.back().timestamp;
  data.xAxis = x.getOutput() * 0.001 * M_PI / 180.0;
  data.yAxis = y.getOutput() * 0.001 * M_PI / 180.0;
  data.zAxis = z.getOutput() * 0.001 * M_PI / 180.0;
  // data.xAxis = (data.xAxis / (float)datasets.size()) * 0.001 * M_PI / 180.0;
  // data.yAxis = (data.yAxis / (float)datasets.size()) * 0.001 * M_PI / 180.0;
  // data.zAxis = (data.zAxis / (float)datasets.size()) * 0.001 * M_PI / 180.0;
}
void UpstreamData::GyroscopeFrame::readData(GyroscopeDataset& dest) {
  std::lock_guard<std::mutex> lock(dataMutex);
  dest = data;
}

GyroscopeFrame::~GyroscopeFrame() {}

void GyroscopeFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  if (iDataSize % datasetBinarySize != 0) {
    std::cout << "Bad Gyroscope frame received. Lenght is mismatched" << std::endl;
    return;
  }

  int dataCount = iDataSize / datasetBinarySize;
  std::lock_guard<std::mutex> lock(dataMutex);

  datasets = std::vector<GyroscopeDataset>(dataCount);

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

void GyroscopeFrame::doTheProcessing() {
  for (int i = 0; i < datasets.size(); i++) {
    // tranforming on mdps - milidegrees per second using board xnucleo iks01a3
    // multipliers are derived from internal board settings for gyroscope with 250dps max range
    datasets[i].xAxis = datasets[i].xAxis * 8.25;
    datasets[i].yAxis = datasets[i].yAxis * 8.25;
    datasets[i].zAxis = datasets[i].zAxis * 8.25;
  }
}

// std::unique_ptr<UpstreamDataType> GyroscopeFrame::getClone()
// {
//   std::unique_ptr<GyroscopeFrame> gyroscopeFrame(new GyroscopeFrame);

//   gyroscopeFrame->protocolIndentificator = protocolIndentificator;
//   gyroscopeFrame->datasetBinarySize = datasetBinarySize;

//   return std::move(gyroscopeFrame);
// }
}  // namespace UpstreamData
}  // namespace Interface