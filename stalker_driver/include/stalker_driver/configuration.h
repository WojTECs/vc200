#pragma once
#include "stalker_driver/DownstreamDataType.h"

namespace Interface {
namespace DownstreamData {
class MainConfigurationFrame : public Interface::DownstreamDataType {
private:
  std::vector<uint8_t> command;
  void doTheProcessing();

public:
  MainConfigurationFrame();
  virtual ~MainConfigurationFrame();

  std::vector<uint8_t> serialize() override;
  void setCommand(std::vector<uint8_t>& in);
  // void deserialize(const uint16_t* msgArray, uint16_t arraySize) override;
};
}  // namespace DownstreamData
}  // namespace Interface
