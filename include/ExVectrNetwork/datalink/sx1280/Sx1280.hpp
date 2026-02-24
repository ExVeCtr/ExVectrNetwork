#ifndef EXVECTRNETWORK_DATALINKSX1280_H_
#define EXVECTRNETWORK_DATALINKSX1280_H_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrHAL/digital_io.hpp"
#include "ExVectrHAL/pin_gpio.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/DatalinkI.hpp"
#include "ExVectrNetwork/physical/HasChannels.hpp"

#include "sx12xxAL/src/SX128XLT.h"

namespace VCTR::network::datalink {

enum SX1280_SF : uint8_t {
  SF_5 = LORA_SF5,
  SF_6 = LORA_SF6,
  SF_7 = LORA_SF7,
  SF_8 = LORA_SF8,
  SF_9 = LORA_SF9,
  SF_10 = LORA_SF10,
  SF_11 = LORA_SF11,
  SF_12 = LORA_SF12
};

enum SX1280_BW : uint8_t {
  BW_200KHz = LORA_BW_0200,
  BW_400KHz = LORA_BW_0400,
  BW_800KHz = LORA_BW_0800,
  BW_1600KHz = LORA_BW_1600,
};

enum SX1280_CR : uint8_t {
  CR_4_5 = LORA_CR_4_5,
  CR_4_6 = LORA_CR_4_6,
  CR_4_7 = LORA_CR_4_7,
  CR_4_8 = LORA_CR_4_8,
  LI_4_5 = LORA_CR_LI_4_5,
  LI_4_6 = LORA_CR_LI_4_6,
  LI_4_8 = LORA_CR_LI_4_8
};

/**
 * @brief A class implementing a datalink layer for the SX1280 LoRa transceiver.
 */
class Datalink_SX1280 : public VCTR::network::datalink::DatalinkI,
                        public VCTR::Core::Task_Periodic {
private:
  ///@brief Maximum length a data frame can be.
  static constexpr size_t dataLinkMaxFrameLength = 200;

  using HandlerFunction = std::function<void()>;

  SX128XLT &lora;

  int16_t receivedDataRSSI_;
  int16_t receivedDataSNR_;

  Core::ListBuffer<uint8_t, dataLinkMaxFrameLength * 5> transmitBuffer_;

  enum class RadioState {
    Idle,
    ChannelBusy,
    Transmitting,
    Receiving,
    WaitingForReceive,
    ActivityDetection
  };
  RadioState radioState_ = RadioState::Idle;

  int64_t transmitStart_ = 0;
  int64_t transmitEnd_ = 0;

  int64_t receiveStart_ = 0;
  int64_t receiveEnd_ = 0;

  int64_t activityDetectionStart_ = 0;
  int64_t activityDetectionEnd_ = 0;

  int64_t channelBusyStart_ = 0;
  int64_t channelBusyEnd_ = 0;

  int64_t idleStart_ = 0;

  float txPower_ = 12;

  bool enableTxRx_ = false;
  bool cadBeforeSend_ = false;

  bool modParamsChanged = true;
  SX1280_SF spreadingFactor = SX1280_SF::SF_10;
  SX1280_BW bandwidth = SX1280_BW::BW_800KHz;
  SX1280_CR codingRate = SX1280_CR::CR_4_8;
  uint32_t freq_hz = 2445000000;

  Core::HandlerGroup<> transmitFinishedHandler_;

  const char *name;

  void logMsg(const char *format, ...) const;
  void debugLog(const char *format, ...) const;

public:
  Datalink_SX1280(SX128XLT &sx1280Driver, const char *name = "RadioSX1280");

  void setName(const char *name) { this->name = name; }

  int16_t lastPacketRSSI() const { return receivedDataRSSI_; }
  int16_t lastPacketSNR() const { return receivedDataSNR_; }

  size_t getBufferFreeSpace() const;
  size_t getMaxPacketSize() const override;
  bool isChannelBlocked() const override;

  void setFrequency(uint32_t freq_hz);
  void setSpreadingFactor(SX1280_SF sf);
  void setBandwidth(SX1280_BW bw);
  void setCodingRate(SX1280_CR cr);
  void setTxPower(float power);

  /**
   * @brief Enables or disables the ability to transmit or receive data.
   * @example For Diversity, we disable all radios accept one for transmitting,
   * otherwise other receivers/ antennas will receive sending radio data.
   */
  void enableTxRx(bool enable);
  void cadBeforeSend(bool enable);

  void clearTransmitBuffer();

  void addTransmitFinishedHandler(HandlerFunction handler);

  void taskInit() override;
  void taskThread() override;
  void taskCheck() override;

  bool transmitDataframe(const DataPacket &dataframe) override;

private:
  void updateModulationParams();

  bool transmitAwaitingData(int64_t threadTime);
  void beginReceive(int timeout = 0);
  void receiveAwaitingData();
};

} // namespace VCTR::network::datalink

#endif