#include "stalker_driver/current_measurement.h"

namespace Interface
{
  namespace UpstreamData
  {
    CurrentMeasurementFrame::CurrentMeasurementFrame()
    {
      protocolIndentificator = uint8_t{0x14};
      datasetBinarySize = 0x4;
    }

    void CurrentMeasurementFrame::readData(CurrentMeasurementDataset &dest)
    {
      std::lock_guard<std::mutex> lock(dataMutex);
      dest = dataAvg;
    }

    CurrentMeasurementFrame::~CurrentMeasurementFrame() {}

    void CurrentMeasurementFrame::deserialize(const uint8_t *iDataStream, const int iDataSize)
    {
      if (iDataSize % datasetBinarySize != 0)
      {
        std::cout << "Bad current measurement frame received. Lenght is mismatched" << std::endl;
        return;
      }

      int dataCount = iDataSize / datasetBinarySize;
      std::lock_guard<std::mutex> lock(dataMutex);
      datasets = std::vector<CurrentIncomingDataset>(dataCount);

      int byteShift = 0;

      for (int i = 0; i < dataCount; i++)
      {
        byteShift = i * datasetBinarySize;
        datasets[i].channel_A = uint16_t((iDataStream[0 + byteShift] << 8) | (iDataStream[1 + byteShift] & 0xFF));
        datasets[i].channel_B = uint16_t((iDataStream[2 + byteShift] << 8) | (iDataStream[3 + byteShift] & 0xFF));
      }
      doTheProcessing()
    }

    void CurrentMeasurementFrame::doTheProcessing()
    {
      int32_t tmp_channel_A = 0;
      int32_t tmp_channel_B = 0;

      for (int i = 0; i < datasets.size(); i++)
      {
        tmp_channel_A += datasets[i].channel_A;
        tmp_channel_B += datasets[i].channel_B;
      }

      dataAvg.channel_A = tmp_channel_A / datasets.size();
      dataAvg.channel_B = tmp_channel_B / datasets.size();
    }

  } // namespace UpstreamData
} // namespace Interface
