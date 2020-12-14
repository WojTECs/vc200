#pragma once

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"
namespace Interface {
namespace DownstreamData {
struct TimerConfigurationCommandDataset {
  int timerID;
  int prescalerValue;
  int counterValue;
  int clockDivider;
};
class TimerConfigurationFrame : public Interface::DownstreamDataType {
private:
  TimerConfigurationCommandDataset command;
  void doTheProcessing();

public:
  TimerConfigurationFrame();
  virtual ~TimerConfigurationFrame();
  void setCommand(TimerConfigurationCommandDataset &in);
  std::vector<uint8_t> serialize();

  // void deserialize(const uint16_t* msgArray, uint16_t arraySize) override;
};
}  // namespace DownstreamData

namespace UpstreamData {
struct ServiceTimeDataset {
  uint16_t IMUTime;
  uint16_t dataSendTime;
  uint16_t EnkoderTime;
  uint16_t PWMTime;
  uint16_t LIDARTime;
  uint16_t UltrasoundTime;
};
class ServiceTimeFrame : public Interface::UpstreamDataType {
private:
  ServiceTimeDataset data;
  void doTheProcessing() override;

public:
  ServiceTimeFrame();
  virtual ~ServiceTimeFrame();

  void readData(ServiceTimeDataset& dest);
  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  // std::unique_ptr<Interface::UpstreamDataType> getClone() override;
};
// namespace UpstreamData

struct TimersDataset {
  uint16_t reg_psc_imu;
  uint16_t reg_arr_imu;
  uint8_t reg_clk_div_imu;
  uint32_t freq_imu;
  uint16_t data_psc;
  uint16_t data_arr;
  uint8_t reg_clk_div_data_send;  // hell knows what all of that means :> Good luck!
  uint32_t freq_data_send;
};
class TimersFrame : public Interface::UpstreamDataType {
private:
  uint16_t reg_psc_imu;
  uint16_t reg_arr_imu;
  uint8_t reg_clk_div_imu;
  uint32_t freq_imu;
  uint16_t data_psc;
  uint16_t data_arr;
  uint8_t reg_clk_div_data_send;  // hell knows what all of that means :> Good luck!
  uint32_t freq_data_send;
  void doTheProcessing() override;

public:
  TimersFrame();
  virtual ~TimersFrame();

  void readData(TimersDataset& dest);
  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  // std::unique_ptr<Interface::UpstreamDataType> getClone() override;
};
// namespace UpstreamData

class TimeSyncFrame : public Interface::UpstreamDataType {
private:
  uint32_t timeSync;
  void doTheProcessing() override;

public:
  TimeSyncFrame();
  virtual ~TimeSyncFrame();
  void readData(uint32_t& dest);
  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  // std::unique_ptr<Interface::UpstreamDataType> getClone() override;
};
}  // namespace UpstreamData
}  // namespace Interface
