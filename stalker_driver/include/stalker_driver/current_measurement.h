#pragma once

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"
#include <cmath>
#include <queue>

namespace Interface {
namespace UpstreamData {
struct CurrentIncomingDataset {
  uint16_t channel_A;
  uint16_t channel_B;
};

struct CurrentSignedDataset {
  int16_t channel_A;
  int16_t channel_B;
};

struct CurrentMeasurementDataset {
  double channel_A;
  double channel_B;
};

class CurrentMeasurementFrame : public UpstreamDataType {
 private:
  CurrentMeasurementDataset dataAvg;
  std::vector<CurrentIncomingDataset> incomingDatasets;
  std::vector<CurrentMeasurementDataset> processedDatasets;

  std::queue<CurrentIncomingDataset> zeroOffsetCalibration;
  CurrentMeasurementDataset linearCoefA;
  CurrentMeasurementDataset linearCoefB;
  CurrentIncomingDataset noiseEpsilon;
  CurrentSignedDataset zeroOffset;
  CurrentIncomingDataset zeroPosition;
  CurrentIncomingDataset ADCAvg;
  double lastPositionA;
  double lastPositionB;

  const uint8_t ADC_RESOLUTION = 12;
  const uint16_t MAX_SIZE = pow(2, ADC_RESOLUTION);
  const uint8_t CALIBRATION_QUEUE_LEN = 10;

  void doTheProcessing() override;

 public:
  CurrentMeasurementFrame();
  virtual ~CurrentMeasurementFrame();
  void readData(CurrentMeasurementDataset &dest, std::vector<CurrentMeasurementDataset> &destVect, double velocityA,
                double velocityB, double positionA, double positionB);
  void deserialize(const uint8_t *iDataStream, const int iDataSize) override;
};
}  // namespace UpstreamData
}  // namespace Interface