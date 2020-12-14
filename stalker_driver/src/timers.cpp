#include <stalker_driver/timers.h>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace Interface {
namespace DownstreamData {
TimerConfigurationFrame::TimerConfigurationFrame() {
  potocolIndentificator = "TimerConfigurationFrame";
  stIdentifier = 0x03;
}

TimerConfigurationFrame::~TimerConfigurationFrame() {
}

std::vector<uint8_t> TimerConfigurationFrame::serialize() {
  std::vector<uint8_t> output(7);
  std::lock_guard<std::mutex> lock(dataMutex);
  output[0] = stIdentifier;
  output[1] = command.timerID;
  output[2] = command.prescalerValue >> 8;
  output[3] = command.prescalerValue & 0xFF;
  output[4] = command.counterValue >> 8;
  output[5] = command.counterValue & 0xFF;
  output[6] = command.clockDivider;

  return output;
}

void TimerConfigurationFrame::setCommand(TimerConfigurationCommandDataset& in) {
  std::lock_guard<std::mutex> lock(dataMutex);
  command = in;
  doTheProcessing();
}

void TimerConfigurationFrame::doTheProcessing() {
}
}  // namespace DownstreamData
namespace UpstreamData {
// ServiceTimeFrame start
ServiceTimeFrame::ServiceTimeFrame() {
  protocolIndentificator = uint8_t{ 0x11 };
  datasetBinarySize = 12;
}

ServiceTimeFrame::~ServiceTimeFrame() {
}
void ServiceTimeFrame::readData(ServiceTimeDataset& dest) {
  std::lock_guard<std::mutex> lock(dataMutex);
  dest = data;
}
void ServiceTimeFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  if (iDataSize != datasetBinarySize) {
    std::cout <<"Bad Time Sync frame received. Length is mismatching"<< std::endl;
    return;
  }
  std::lock_guard<std::mutex> lock(dataMutex);
  data.IMUTime = (iDataStream[0] << 8) | (iDataStream[1] & 0xFF);
  data.dataSendTime = (iDataStream[2] << 8) | (iDataStream[3] & 0xFF);
  data.EnkoderTime = (iDataStream[4] << 8) | (iDataStream[5] & 0xFF);
  data.PWMTime = (iDataStream[6] << 8) | (iDataStream[7] & 0xFF);
  data.LIDARTime = (iDataStream[8] << 8) | (iDataStream[9] & 0xFF);
  data.UltrasoundTime = (iDataStream[10] << 8) | (iDataStream[11] & 0xFF);
  doTheProcessing();
}

void ServiceTimeFrame::doTheProcessing() {
}

// std::unique_ptr<Interface::UpstreamDataType> ServiceTimeFrame::getClone() {
//   std::unique_ptr<ServiceTimeFrame> serviceTimeFrame(new ServiceTimeFrame);

//   serviceTimeFrame->protocolIndentificator = protocolIndentificator;
//   serviceTimeFrame->datasetBinarySize = datasetBinarySize;

//   return std::move(serviceTimeFrame);
// }
// ServiceTimeFrame end

// TimersFrame start
TimersFrame::TimersFrame() {
  protocolIndentificator = uint8_t{ 0x03 };
  datasetBinarySize = 18;  // Unknown yet therefore can't be implemented
}

TimersFrame::~TimersFrame() {
}
void TimersFrame::readData(TimersDataset& dest) {
  dataMutex.lock();
  dest.reg_psc_imu = reg_psc_imu;
  dest.reg_arr_imu = reg_arr_imu;
  dest.reg_clk_div_imu = reg_clk_div_imu;
  dest.freq_imu = freq_imu;
  dest.data_psc = data_psc;
  dest.data_arr = data_arr;
  dest.reg_clk_div_data_send = reg_clk_div_data_send;
  dest.freq_data_send = freq_data_send;
  dataMutex.unlock();
}
void TimersFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  dataMutex.lock();
  reg_psc_imu = (iDataStream[0] << 8) | (iDataStream[1] & 0xFF);
  reg_arr_imu = (iDataStream[2] << 8) | (iDataStream[3] & 0xFF);
  reg_clk_div_imu = iDataStream[4] & 0xFF;
  freq_imu = (iDataStream[5] << 24) | (iDataStream[6] << 16) | (iDataStream[7] << 8) | (iDataStream[8] & 0xFF);
  data_psc = (iDataStream[9] << 8) | (iDataStream[10] & 0xFF);
  data_arr = (iDataStream[11] << 8) | (iDataStream[12] & 0xFF);
  reg_clk_div_data_send = iDataStream[13] & 0xFF;
  freq_data_send =
      (iDataStream[14] << 24) | (iDataStream[15] << 16) | (iDataStream[16] << 8) | (iDataStream[17] & 0xFF);
  doTheProcessing();
  dataMutex.unlock();
}

void TimersFrame::doTheProcessing() {
}

// std::unique_ptr<Interface::UpstreamDataType> TimersFrame::getClone() {
//   std::unique_ptr<TimersFrame> timersFrame(new TimersFrame);

//   timersFrame->protocolIndentificator = protocolIndentificator;
//   timersFrame->datasetBinarySize = datasetBinarySize;

//   return std::move(timersFrame);
// }

// TimersFrame end

// TimeSyncFrame start
TimeSyncFrame::TimeSyncFrame() {
  protocolIndentificator = uint8_t{ 0xFF };
  datasetBinarySize = 4;
}

TimeSyncFrame::~TimeSyncFrame() {
}

void TimeSyncFrame::readData(uint32_t& dest) {
  dataMutex.lock();
  dest = timeSync;
  dataMutex.unlock();
}
void TimeSyncFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  if (iDataSize != datasetBinarySize) {
    std::cout <<"Bad Time Sync frame received. Length is mismatching"<<std::endl;
    return;
  }
  dataMutex.lock();
  timeSync = (iDataStream[0] << 24) | (iDataStream[1] << 16) | (iDataStream[2] << 8) | (iDataStream[3] & 0xFF);
  doTheProcessing();
  dataMutex.unlock();
}

void TimeSyncFrame::doTheProcessing() {
}

// std::unique_ptr<Interface::UpstreamDataType> TimeSyncFrame::getClone() {
//   std::unique_ptr<TimeSyncFrame> timeSyncFrame(new TimeSyncFrame);

//   timeSyncFrame->protocolIndentificator = protocolIndentificator;
//   timeSyncFrame->datasetBinarySize = datasetBinarySize;

//   return std::move(timeSyncFrame);
// }
// TimeSyncFrame end
}  // namespace UpstreamData
}  // namespace Interface