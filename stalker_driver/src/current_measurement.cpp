#include "stalker_driver/current_measurement.h"

namespace Interface
{
  namespace UpstreamData
  {
    CurrentMeasurementFrame::CurrentMeasurementFrame()
    {
      protocolIndentificator = uint8_t{0x13};
      datasetBinarySize = 0x4;
    }

    void CurrentMeasurementFrame::readData(CurrentMeasurementDataset& dest)
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

    // std::cout << "cmd.rightSidePWM: " << cmd.rightSidePWM << std::endl;
    // std::cout << "leftVelocityCommand: " << leftVelocityCommand << std::endl;
    // std::cout << "rightVelocityCommand: " << rightVelocityCommand << std::endl;
      //std::cout << "\x1B[2J\x1B[H";
      for (int i = 0; i < dataCount; i++)
      {
        byteShift = i * datasetBinarySize;
        datasets[i].channel_A = uint16_t((iDataStream[1 + byteShift] << 8) | (iDataStream[0 + byteShift] & 0xFF)) & 0xFFFF;
        datasets[i].channel_B = uint16_t((iDataStream[3 + byteShift] << 8) | (iDataStream[2 + byteShift] & 0xFF)) & 0xFFFF;

        // if (datasets[i].channel_A  > 1360 || datasets[i].channel_A  < 1100 ||
        //     datasets[i].channel_B  > 1360 || datasets[i].channel_B  < 1100 ) 
        // {
        //   std::cout << i << "\tCurrent\tA:\t" << datasets[i].channel_A << "\t\tB:\t" << datasets[i].channel_B << std::endl;
        // }
      }
      
      doTheProcessing();


      //std::cout << "\x1B[2J\x1B[H";
      //std::cout << "Current CH_A: " << dataAvg.channel_A << " \t CH_B: " << dataAvg.channel_B << std::endl;
    }

    void CurrentMeasurementFrame::doTheProcessing()
    {
      int32_t tmp_channel_A = 0;
      int32_t tmp_channel_B = 0;

    
      //std::cout << "---------------------------------------------------------------------------------------" << std::endl;

      for (int i = 0; i < datasets.size(); i++)
      {
        tmp_channel_A += datasets[i].channel_A;
        tmp_channel_B += datasets[i].channel_B;

        //std::cout << i << "\tCurrent\tA:\t" << datasets[i].channel_A << "\t\tB:\t" << datasets[i].channel_B << std::endl;
      }

      dataAvg.channel_A = ((double)tmp_channel_A) / datasets.size();
      dataAvg.channel_B = ((double)tmp_channel_B) / datasets.size();
    }

  } // namespace UpstreamData
} // namespace Interface
