#pragma once
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "stalker_driver/UpstreamDataType.h"
#include "stalker_driver/DownstreamDataType.h"
namespace Interface {
namespace DownstreamData {
// This message type is used for SWORD and RAPTOR remotely controlled platforms and those that have separate turn and
// propulsion servo
struct MovementCommandDataset {
  int leftDirection;
  int rightDirection;
  int leftSidePWM;
  int rightSidePWM;
  int timeToDrive;
  int shallQueue;
};
class MovementOrderLeftRightFrame : public Interface::DownstreamDataType {
private:
  MovementCommandDataset command;
  void doTheProcessing() override;

public:
  MovementOrderLeftRightFrame();
  virtual ~MovementOrderLeftRightFrame();

  void setCommand(MovementCommandDataset &in);
  std::vector<uint8_t> serialize();
  // void deserialize(const uint16_t* msgArray, uint16_t arraySize) override;
};

}  // namespace DownstreamData

namespace UpstreamData {
struct MovementInformationDataset {
  int rightWheelDirection;
  int leftWheelDirection;
  int rightWheelPwm;
  int leftWheelPwm;
  int remainedTimeToDrive;
  int howManyQueued;
};
class MovementInformationLeftRightFrame : public Interface::UpstreamDataType {
private:
  MovementInformationDataset data;

public:
  MovementInformationLeftRightFrame();
  virtual ~MovementInformationLeftRightFrame();

  void readData(MovementInformationDataset& dest);
  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  void doTheProcessing() override;
  // std::unique_ptr<Interface::UpstreamDataType> getClone() override;
};

union floatUnion {
  float value;
  int32_t int_value;
  uint8_t array[sizeof(float)];
};

struct EncoderDataset {
  uint8_t leftRotationDirection;
  uint8_t rightRotationDirection;

  floatUnion leftSideVelocity;
  floatUnion rightSideVelocity;

  floatUnion leftSideDistance;
  floatUnion rightSideDistance;
};

class EncoderFrame : public Interface::UpstreamDataType {
private:
  EncoderDataset data;

public:
  void readData(EncoderDataset& dest);
  EncoderFrame();
  virtual ~EncoderFrame();

  void deserialize(const uint8_t* iDataStream, const int iDataSize) override;
  void doTheProcessing() override;
  // std::unique_ptr<Interface::UpstreamDataType> getClone() override;
};
}  // namespace UpstreamData

}  // namespace Interface

