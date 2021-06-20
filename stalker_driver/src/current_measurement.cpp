#include "stalker_driver/current_measurement.h"

namespace Interface
{
  namespace UpstreamData
  {
    CurrentMeasurementFrame::CurrentMeasurementFrame()
    {
      protocolIndentificator = uint8_t{0x13};
      datasetBinarySize = 0x4;

      noiseEpsilon = {10, 10};
      linearCoefA = {0.1, 0.1};
      linearCoefB = {40.0, 40.0};

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
      incomingDatasets = std::vector<CurrentIncomingDataset>(dataCount);
      processedDatasets = std::vector<CurrentMeasurementDataset>(incomingDatasets.size());

      int byteShift = 0;

      //std::cout << "\x1B[2J\x1B[H";
      for (int i = 0; i < dataCount; i++)
      {
        byteShift = i * datasetBinarySize;
        incomingDatasets[i].channel_A = uint16_t((iDataStream[1 + byteShift] << 8) | (iDataStream[0 + byteShift] & 0xFF));
        incomingDatasets[i].channel_B = uint16_t((iDataStream[3 + byteShift] << 8) | (iDataStream[2 + byteShift] & 0xFF));

        if (incomingDatasets[i].channel_A > MAX_SIZE)
        {
          std::cout << "Bad current measurement frame received. Data for channel A is greater than ADC resolution" << std::endl;
          return;
        }

        if (incomingDatasets[i].channel_B > MAX_SIZE)
        {
          std::cout << "Bad current measurement frame received. Data for channel B is greater than ADC resolution" << std::endl;
          return;
        }

        // if (incomingDatasets[i].channel_A  > 1360 || incomingDatasets[i].channel_A  < 1100 ||
        //     incomingDatasets[i].channel_B  > 1360 || incomingDatasets[i].channel_B  < 1100 ) 
        // {
        //   std::cout << i << "\tCurrent\tA:\t" << incomingDatasets[i].channel_A << "\t\tB:\t" << incomingDatasets[i].channel_B << std::endl;
        // }
      }
      
      doTheProcessing();


      //std::cout << "\x1B[2J\x1B[H";
      //std::cout << "Current CH_A: " << dataAvg.channel_A << " \t CH_B: " << dataAvg.channel_B << std::endl;
    }

    void CurrentMeasurementFrame::doTheProcessing()
    {
      dataAvg = {0.0, 0.0};
    
      //std::cout << "---------------------------------------------------------------------------------------" << std::endl;

      for (int i = 0; i < incomingDatasets.size(); i++)
      {
        processedDatasets[i].channel_A = linearCoefA.channel_A * incomingDatasets[i].channel_A + linearCoefB.channel_A;
        processedDatasets[i].channel_B = linearCoefA.channel_B * incomingDatasets[i].channel_B + linearCoefB.channel_B;

        dataAvg.channel_A += processedDatasets[i].channel_A;
        dataAvg.channel_B += processedDatasets[i].channel_B;

        //std::cout << i << "\Processed current\tA:\t" << processedDatasets[i].channel_A << "\t\tB:\t" << processedDatasets[i].channel_B << std::endl;
      }

      dataAvg.channel_A = ((double)dataAvg.channel_A) / incomingDatasets.size();
      dataAvg.channel_B = ((double)dataAvg.channel_B) / incomingDatasets.size();
    }

  } // namespace UpstreamData
} // namespace Interface
