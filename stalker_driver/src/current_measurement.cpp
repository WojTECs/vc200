#include "stalker_driver/current_measurement.h"

namespace Interface {
namespace UpstreamData {
CurrentMeasurementFrame::CurrentMeasurementFrame() {
  protocolIndentificator = uint8_t{0x13};
  datasetBinarySize = 0x4;

  noiseEpsilon = {15, 15};
  linearCoefA = {000.004107459, 000.004107459};
  linearCoefB = {-012.482455249, -012.412628453};
  zeroOffset = {0, 0};

  zeroPosition = {(uint16_t)(-linearCoefB.channel_A / (double)linearCoefA.channel_A),
                  (uint16_t)(-linearCoefB.channel_B / (double)linearCoefA.channel_B)};

  zeroOffsetCalibration = std::queue<CurrentIncomingDataset>();
}

void CurrentMeasurementFrame::readData(CurrentMeasurementDataset &dest,
                                       std::vector<CurrentMeasurementDataset> &destVect, double velocityA,
                                       double velocityB, double positionA, double positionB) {
  std::lock_guard<std::mutex> lock(dataMutex);

  if (positionA == lastPositionA && positionB == lastPositionB) {
    zeroOffsetCalibration.push(ADCAvg);

    if (zeroOffsetCalibration.size() > CALIBRATION_QUEUE_LEN - 1) {
      zeroOffset = {0, 0};

      for (size_t i = 0; i < CALIBRATION_QUEUE_LEN; i++) {
        zeroOffset.channel_A += (int16_t)zeroPosition.channel_A - (int16_t)zeroOffsetCalibration.front().channel_A;
        zeroOffset.channel_B += (int16_t)zeroPosition.channel_B - (int16_t)zeroOffsetCalibration.front().channel_B;
        // std::cout << i << " Zero offset\tA: " << zeroOffset.channel_A << "\tB: " << zeroOffset.channel_B <<
        // std::endl;

        zeroOffsetCalibration.pop();
      }

      //zeroOffset.channel_A = (int16_t)((double)zeroOffset.channel_A / (double)CALIBRATION_QUEUE_LEN);
      //zeroOffset.channel_B = (int16_t)((double)zeroOffset.channel_B / (double)CALIBRATION_QUEUE_LEN);

      // zeroPosition.channel_A += zeroOffset.channel_A;
      // zeroPosition.channel_B += zeroOffset.channel_B;
      // std::cout << "Current measurement zero offset calibration: A: " << zeroOffset.channel_A << " B: " <<
      // zeroOffset.channel_B << std::endl;
    }
  } else {
    zeroOffsetCalibration = std::queue<CurrentIncomingDataset>();
  }

  lastPositionA = positionA;
  lastPositionB = positionB;

  dest = dataAvg;
  destVect = processedDatasets;
}

CurrentMeasurementFrame::~CurrentMeasurementFrame() {}

void CurrentMeasurementFrame::deserialize(const uint8_t *iDataStream, const int iDataSize) {
  if (iDataSize % datasetBinarySize != 0) {
    std::cout << "Bad current measurement frame received. Lenght is mismatched" << std::endl;
    return;
  }

  int dataCount = iDataSize / datasetBinarySize;
  std::lock_guard<std::mutex> lock(dataMutex);
  incomingDatasets = std::vector<CurrentIncomingDataset>(dataCount);
  processedDatasets = std::vector<CurrentMeasurementDataset>(incomingDatasets.size());

  int byteShift = 0;

  // std::cout << "\x1B[2J\x1B[H";
  for (int i = 0; i < dataCount; i++) {
    byteShift = i * datasetBinarySize;

    incomingDatasets[i].channel_A = uint16_t((iDataStream[1 + byteShift] << 8) | (iDataStream[0 + byteShift] & 0xFF));
    incomingDatasets[i].channel_B = uint16_t((iDataStream[3 + byteShift] << 8) | (iDataStream[2 + byteShift] & 0xFF));

    if (incomingDatasets[i].channel_A > MAX_SIZE) {
      std::cout << "Bad current measurement frame received. Data for channel A is greater than ADC resolution"
                << std::endl;
      return;
    }

    if (incomingDatasets[i].channel_B > MAX_SIZE) {
      std::cout << "Bad current measurement frame received. Data for channel B is greater than ADC resolution"
                << std::endl;
      return;
    }
  }

  doTheProcessing();
}

void CurrentMeasurementFrame::doTheProcessing() {
  dataAvg = {0.0, 0.0};
  int32_t tmpA = 0;
  int32_t tmpB = 0;
  zeroOffset = {0, 0};

  // std::cout << "---------------------------------------------------------------------------------------" <<
  // std::endl;

  for (int i = 0; i < incomingDatasets.size(); i++) {
    tmpA += incomingDatasets[i].channel_A;
    tmpB += incomingDatasets[i].channel_B;
    
    incomingDatasets[i].channel_A = (int16_t)incomingDatasets[i].channel_A + zeroOffset.channel_A;
    incomingDatasets[i].channel_B = (int16_t)incomingDatasets[i].channel_B + zeroOffset.channel_B;

    processedDatasets[i].channel_A = linearCoefA.channel_A * incomingDatasets[i].channel_A + linearCoefB.channel_A;
    processedDatasets[i].channel_B = linearCoefA.channel_B * incomingDatasets[i].channel_B + linearCoefB.channel_B;

    if ((incomingDatasets[i].channel_A < (int16_t)zeroPosition.channel_A + (int16_t)noiseEpsilon.channel_A) &&
        (incomingDatasets[i].channel_A > (int16_t)zeroPosition.channel_A - (int16_t)noiseEpsilon.channel_A)) {
      processedDatasets[i].channel_A = 0.0;
    }

    if ((incomingDatasets[i].channel_B < (int16_t)zeroPosition.channel_B + (int16_t)noiseEpsilon.channel_B) &&
        (incomingDatasets[i].channel_B > (int16_t)zeroPosition.channel_B - (int16_t)noiseEpsilon.channel_B)) {
      processedDatasets[i].channel_B = 0.0;
    }

    dataAvg.channel_A += (double)processedDatasets[i].channel_A;
    dataAvg.channel_B += (double)processedDatasets[i].channel_B;

    // std::cout << i << "Processed current\tA:\t" << processedDatasets[i].channel_A << "\t\tB:\t" <<
    // processedDatasets[i].channel_B << std::endl;
  }

  ADCAvg.channel_A = (uint16_t)((double)tmpA / incomingDatasets.size());
  ADCAvg.channel_B = (uint16_t)((double)tmpB / incomingDatasets.size());

  dataAvg.channel_A = ((double)dataAvg.channel_A) / incomingDatasets.size();
  dataAvg.channel_B = ((double)dataAvg.channel_B) / incomingDatasets.size();

  // if ((dataAvg.channel_A < linearCoefA.channel_A * (zeroPosition.channel_A + noiseEpsilon.channel_A) +
  // linearCoefB.channel_A) &&
  //     (dataAvg.channel_A > linearCoefA.channel_A * (zeroPosition.channel_A - noiseEpsilon.channel_A) +
  //     linearCoefB.channel_A))
  // {
  //   dataAvg.channel_A = 0.0;
  // }

  // if ((dataAvg.channel_B < linearCoefA.channel_B * (zeroPosition.channel_B + noiseEpsilon.channel_B) +
  // linearCoefB.channel_B) &&
  //     (dataAvg.channel_B > linearCoefA.channel_B * (zeroPosition.channel_B - noiseEpsilon.channel_B) +
  //     linearCoefB.channel_B))
  // {
  //   dataAvg.channel_B = 0.0;
  // }

  // std::cout << "AVG current\tA: " << dataAvg.channel_A << "\tADC: " << ADCAvg.channel_A << " (" << ADCAvg.channel_A *
  // (3.3 / 4096) << "V)"
  //           << "\tB: " << dataAvg.channel_B << "\tADC: " << ADCAvg.channel_B << " (" << ADCAvg.channel_B * (3.3 /
  //           4096) << "V)" << std::endl;

  // std::cout << "Zero current\tA: " << zeroPosition.channel_A << "\tnoiseEps: " << noiseEpsilon.channel_A
  //           << "\tB: " << zeroPosition.channel_B << "\tnoiseEps: " << noiseEpsilon.channel_B << std::endl;
}
}  // namespace UpstreamData
}  // namespace Interface
