#include "stalker_driver/configuration.h"
#include <boost/exception/diagnostic_information.hpp>

namespace Interface {
namespace DownstreamData {
MainConfigurationFrame::MainConfigurationFrame() {
  potocolIndentificator = "MainConfigurationFrame";
}

MainConfigurationFrame::~MainConfigurationFrame() {
  stIdentifier = 0x00;
}

std::vector<uint8_t> MainConfigurationFrame::serialize() {
  std::vector<uint8_t> output;
  std::lock_guard<std::mutex> lock(dataMutex);
  output.push_back(stIdentifier);  // ID
  output.push_back(command.size());

  output.insert(output.end(), command.begin(), command.end());
  command.clear();

  return output;
}

void MainConfigurationFrame::setCommand(std::vector<uint8_t>& in) {
  std::lock_guard<std::mutex> lock(dataMutex);
  for (uint8_t& i : in) {
    command.push_back(i);
  }
  doTheProcessing();
}

void MainConfigurationFrame::doTheProcessing() {
}
}  // namespace DownstreamData
}  // namespace Interface