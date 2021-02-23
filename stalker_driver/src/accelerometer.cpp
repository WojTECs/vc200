#include "stalker_driver/accelerometer.h"

#include <algorithm>
#include <boost/exception/diagnostic_information.hpp>
#include <iostream>
#include <numeric>

namespace Interface {
namespace DownstreamData {
AccelerometerFrame::AccelerometerFrame() {
  potocolIndentificator = "AccelerometerFrame";
  stIdentifier = 0x04;
}

AccelerometerFrame::~AccelerometerFrame() {}
std::vector<uint8_t> AccelerometerFrame::serialize() {
  std::vector<uint8_t> output(3);
  std::lock_guard<std::mutex> lock(dataMutex);
  output[0] = stIdentifier;
  output[1] = command.registryAddress;
  output[2] = command.registryValue;
  return output;
}

void AccelerometerFrame::setCommand(AccelerometerCommandDataset& in) {
  std::lock_guard<std::mutex> lock(dataMutex);
  command = in;
  doTheProcessing();
}

void AccelerometerFrame::doTheProcessing() {}
}  // namespace DownstreamData

namespace UpstreamData {
void AccelerometerFrame::filter() {
  data = {};
  for (auto const& dataset : datasets) {
    // data.xAxis += dataset.xAxis;
    x.filter(dataset.xAxis);
    // data.yAxis += dataset.yAxis;
    y.filter(dataset.yAxis);
    // data.zAxis += dataset.zAxis;
    z.filter(dataset.zAxis);
  }
  data.timestamp = datasets.back().timestamp;

  data.xAxis = x.getOutput() * 0.00980665;
  data.yAxis = y.getOutput() * 0.00980665;
  data.zAxis = z.getOutput() * 0.00980665;

  // data.xAxis = (data.xAxis / (float)datasets.size()) * 0.00980665;
  // data.yAxis = (data.yAxis / (float)datasets.size()) * 0.00980665;
  // data.zAxis = (data.zAxis / (float)datasets.size()) * 0.00980665;
}
AccelerometerFrame::AccelerometerFrame() : x(0.05, 0), y(0.05, 0), z(0.05, 0) {
  protocolIndentificator = uint8_t{0x04};
  datasetBinarySize = 10;
}

AccelerometerFrame::~AccelerometerFrame() {}

void AccelerometerFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  if (iDataSize % datasetBinarySize != 0) {
    // TODO - when the frame is being cut by buffers
    std::cout << "Bad Accelerometer frame received: wrong data lenght" << std::endl;
    return;
  }

  int dataCount = iDataSize / datasetBinarySize;
  std::lock_guard<std::mutex> lock(dataMutex);
  datasets = std::vector<AccelerometerDataset>(dataCount);

  int byteShift = 0;

  for (int i = 0; i < dataCount; i++) {
    byteShift = i * datasetBinarySize;
    datasets[i].xAxis = int16_t((iDataStream[0 + byteShift] << 8) | (iDataStream[1 + byteShift] & 0xFF));
    datasets[i].yAxis = int16_t((iDataStream[2 + byteShift] << 8) | (iDataStream[3 + byteShift] & 0xFF));
    datasets[i].zAxis = int16_t((iDataStream[4 + byteShift] << 8) | (iDataStream[5 + byteShift] & 0xFF));
    datasets[i].timestamp =
        (iDataStream[6 + byteShift] << 24) |
        (iDataStream[7 + byteShift] << 16) + (iDataStream[8 + byteShift] << 8) + (iDataStream[9 + byteShift] & 0xFF);
  }
  doTheProcessing();
  filter();
  // print();
}
void AccelerometerFrame::print() {
  std::cout << std::setw(18) << std::left << ("[X]: " + std::to_string(data.xAxis) + " ");
  std::cout << std::setw(18) << std::left << ("[Y]: " + std::to_string(data.yAxis) + " ");
  std::cout << std::setw(18) << std::left << ("[Z]: " + std::to_string(data.zAxis) + " ") << "\n";
}
void AccelerometerFrame::doTheProcessing() {
  for (int i = 0; i < datasets.size(); i++) {
    // tranforming on mg - miligravitation using board xnucleo cd (like earth gravitation 9.81 m/s)
    // multipliers are derived from internal board settings for accelerometer with 2g max range
    datasets[i].xAxis = datasets[i].xAxis * 0.061;
    datasets[i].yAxis = datasets[i].yAxis * 0.061;
    datasets[i].zAxis = datasets[i].zAxis * 0.061;
  }
}

void AccelerometerFrame::readData(AccelerometerDataset& d) {
  std::lock_guard<std::mutex> lock(dataMutex);
  d = data;
}

// std::unique_ptr<UpstreamDataType> AccelerometerFrame::getClone() {
//   std::unique_ptr<AccelerometerFrame> accelerometerFrame(new AccelerometerFrame);

//   accelerometerFrame->protocolIndentificator = protocolIndentificator;
//   accelerometerFrame->datasetBinarySize = datasetBinarySize;

//   return std::move(accelerometerFrame);
// }
}  // namespace UpstreamData
}  // namespace Interface