#pragma once

#include <memory>
#include <vector>
#include <mutex>

namespace Interface
{
class DownstreamDataType
{
private:
protected:
  int stIdentifier;
  std::string potocolIndentificator;
  std::mutex dataMutex;
public:
  virtual ~DownstreamDataType()
  {
  }
  int getProtocolIdentificator() const
  {
    return stIdentifier;
  }
  virtual std::vector<uint8_t> serialize() = 0;
  virtual void doTheProcessing() = 0;
};

}  // namespace Interface
