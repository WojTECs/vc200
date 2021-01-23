#include "stalker_driver/motor_controller.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"

namespace Interface {
namespace DownstreamData {
MovementOrderLeftRightFrame::MovementOrderLeftRightFrame() {
  stIdentifier = 0x02;
  potocolIndentificator = "MovementFrame";
}

MovementOrderLeftRightFrame::~MovementOrderLeftRightFrame() {
}

std::vector<uint8_t> MovementOrderLeftRightFrame::serialize() {
  std::vector<uint8_t> output(9);
  std::lock_guard<std::mutex> lock(dataMutex);
  output[0] = stIdentifier;  // ID
  output[1] = (command.leftDirection << 4) | (command.rightDirection & 0x0F);
  output[2] = command.leftSidePWM >> 8;
  output[3] = command.leftSidePWM & 0xFF;
  output[4] = command.rightSidePWM >> 8;
  output[5] = command.rightSidePWM & 0xFF;
  output[6] = command.timeToDrive >> 8;
  output[7] = command.timeToDrive & 0xFF;
  output[8] = command.shallQueue;
  return output;
}

void MovementOrderLeftRightFrame::setCommand(MovementCommandDataset& in) {
  std::lock_guard<std::mutex> lock(dataMutex);
  command = in;
  doTheProcessing();
}

void MovementOrderLeftRightFrame::doTheProcessing() {
}

}  // namespace DownstreamData

namespace UpstreamData {
EncoderFrame::EncoderFrame() {
  protocolIndentificator = uint8_t{ 0x07 };
  datasetBinarySize = 17;
}

EncoderFrame::~EncoderFrame() {
}

void EncoderFrame::readData(EncoderDataset& dest) {
  std::lock_guard<std::mutex> lock(dataMutex);
  dest = data;
}

void EncoderFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  if (iDataSize != datasetBinarySize) {
    std::cout << "Bad Encoder frame received. Length is mismatching" << std::endl;
    return;
  }
  std::lock_guard<std::mutex> lock(dataMutex);
  data.leftRotationDirection = (iDataStream[0] >> 4) & 0x0F;
  data.rightRotationDirection = (iDataStream[0]) & 0xF;
  data.leftSideVelocity.array[3] = iDataStream[1];
  data.leftSideVelocity.array[2] = iDataStream[2];
  data.leftSideVelocity.array[1] = iDataStream[3];
  data.leftSideVelocity.array[0] = iDataStream[4];
  data.leftSideDistance.array[3] = iDataStream[5];
  data.leftSideDistance.array[2] = iDataStream[6];
  data.leftSideDistance.array[1] = iDataStream[7];
  data.leftSideDistance.array[0] = iDataStream[8];
  data.rightSideVelocity.array[3] = iDataStream[9];
  data.rightSideVelocity.array[2] = iDataStream[10];
  data.rightSideVelocity.array[1] = iDataStream[11];
  data.rightSideVelocity.array[0] = iDataStream[12];
  data.rightSideDistance.array[3] = iDataStream[13];
  data.rightSideDistance.array[2] = iDataStream[14];
  data.rightSideDistance.array[1] = iDataStream[15];
  data.rightSideDistance.array[0] = iDataStream[16];

  doTheProcessing();
}

void EncoderFrame::doTheProcessing() {
}

// std::unique_ptr<Interface::UpstreamDataType> EncoderFrame::getClone() {
//   std::unique_ptr<EncoderFrame> encoderFrame(new EncoderFrame);

//   encoderFrame->protocolIndentificator = protocolIndentificator;
//   encoderFrame->datasetBinarySize = datasetBinarySize;

//   return std::move(encoderFrame);
// }

/////////////

MovementInformationLeftRightFrame::MovementInformationLeftRightFrame() {
  protocolIndentificator = uint8_t{ 0x02 };
  datasetBinarySize = 8;
}

MovementInformationLeftRightFrame::~MovementInformationLeftRightFrame() {
}

void MovementInformationLeftRightFrame::deserialize(const uint8_t* iDataStream, const int iDataSize) {
  std::lock_guard<std::mutex> lock(dataMutex);
  data.leftWheelDirection = (iDataStream[0] >> 4) & 0x0F;
  data.rightWheelDirection = iDataStream[0] & 0x0F;
  data.leftWheelPwm = (iDataStream[1] << 8) | (iDataStream[2] & 0xFF);
  data.rightWheelPwm = (iDataStream[3] << 8) | (iDataStream[4] & 0xFF);
  data.remainedTimeToDrive = (iDataStream[5] << 8) | (iDataStream[6] & 0xFF);
  data.howManyQueued = iDataStream[7];
  doTheProcessing();
}
void MovementInformationLeftRightFrame::readData(MovementInformationDataset& dest) {
  std::lock_guard<std::mutex> lock(dataMutex);
  dest = data;
}
void MovementInformationLeftRightFrame::doTheProcessing() {
}

// std::unique_ptr<Interface::UpstreamDataType> MovementInformationLeftRightFrame::getClone() {
//   std::unique_ptr<MovementInformationLeftRightFrame> pwmFrame(new MovementInformationLeftRightFrame);

//   pwmFrame->protocolIndentificator = protocolIndentificator;
//   pwmFrame->datasetBinarySize = datasetBinarySize;

//   return std::move(pwmFrame);
// }

}  // namespace UpstreamData
}  // namespace Interface