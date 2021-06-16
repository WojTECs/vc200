#pragma once

#include "stalker_driver/DownstreamDataType.h"
#include "stalker_driver/UpstreamDataType.h"

namespace Interface
{
  namespace UpstreamData
  {
    struct CurrentIncomingDataset
    {
      uint16_t channel_A;
      uint16_t channel_B;
    };

    struct CurrentMeasurementDataset
    {
      double channel_A;
      double channel_B;
    };

    class CurrentMeasurementFrame : public UpstreamDataType
    {
    private:
      CurrentMeasurementDataset dataAvg;
      std::vector<CurrentIncomingDataset> datasets;
      // void filter();
      void doTheProcessing() override;

    public:
      CurrentMeasurementFrame();
      virtual ~CurrentMeasurementFrame();
      void readData(CurrentMeasurementDataset& dest);
      void deserialize(const uint8_t *iDataStream, const int iDataSize) override;
    };
  } // namespace UpstreamData
} // namespace Interface