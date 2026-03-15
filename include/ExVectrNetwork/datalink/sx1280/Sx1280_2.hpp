#ifndef EXVECTRNETWORK_DATALINKSX1280_V2_H_
#define EXVECTRNETWORK_DATALINKSX1280_V2_H_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrHAL/digital_io.hpp"
#include "ExVectrHAL/pin_gpio.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
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
class Datalink_SX1280_V2 : public VCTR::network::datalink::RadioI,
                           public VCTR::Core::Task_Periodic {
private:
  ///@brief Maximum length a data frame can be.
  static constexpr size_t dataLinkMaxFrameLength = 200;

  static constexpr uint8_t numChannels = 10;
  static constexpr uint32_t minFreq = 2.41e9;
  static constexpr uint32_t maxFreq = 2.47e9;
  static constexpr uint32_t channelFrequencySpacing =
      (maxFreq - minFreq) / (numChannels - 1);

  using HandlerFunction = std::function<void()>;

  SX128XLT &lora;

  int16_t receivedDataRSSI;
  int16_t receivedDataSNR;

  size_t transmitPacketSize = 0;

  int64_t currentThreadStartTime;

  int64_t channelBusyStart;
  int64_t receiveStart;
  int64_t transmitStart;

  enum class RadioState {
    Idle,
    CAD,
    Receiving,
    Transmitting,
  };
  RadioState radioState = RadioState::Idle;

  enum class RadioEvent {
    None,
    CAD,
    CADDone,
    ReceiveDone,
    TransmitDone,
    RXTimeout,
    TXTimeout,
    HeaderError,
    CRCError,
  };
  Core::ListArray<RadioEvent> events;

  int8_t maxTxPowerDBm = 12;
  int8_t txPower = 0;
  int8_t paDbm = 0;

  bool enableTxRx = false;
  bool cadBeforeSend = false;

  bool modParamsChanged = true;
  bool freqParamChanged = true;
  SX1280_SF spreadingFactor = SX1280_SF::SF_8;
  SX1280_BW bandwidth = SX1280_BW::BW_800KHz;
  SX1280_CR codingRate = SX1280_CR::LI_4_8;
  uint32_t freq_hz = minFreq;

  uint8_t currentChannel = 0;

  Core::HandlerGroup<> transmitFinishedHandler;

public:
  Datalink_SX1280_V2(SX128XLT &sx1280Driver);

  int16_t lastPacketRSSI() const { return receivedDataRSSI; }
  int16_t lastPacketSNR() const { return receivedDataSNR; }

  size_t getMaxPacketSize() const override;
  bool isChannelBlocked() const override;

  void setFrequency(uint32_t freq_hz);
  void setSpreadingFactor(SX1280_SF sf);
  void setBandwidth(SX1280_BW bw);
  void setCodingRate(SX1280_CR cr);
  void setTxPower(int8_t power);
  void setTxMaxPower(int8_t maxTxPowerDBm);

  // Sets the number of db the PA adds to the output power.
  // This is used to change sx1280 output power to reach the desired Tx power.
  void setPAdbm(uint8_t paDbm);
  /**
   * @brief Enables or disables the ability to transmit or receive data.
   * @example For Diversity, we disable all radios accept one for transmitting,
   * otherwise other receivers/ antennas will receive sending radio data.
   */
  void setEnableTxRx(bool enable);

  void addTransmitFinishedHandler(HandlerFunction handler);

  void taskInit() override;
  void taskThread() override;
  void taskCheck() override;

  bool transmitDataframe(const DataPacket &dataframe) override;

  size_t getNumChannels() const override;
  size_t getCurrentChannel() const override;
  void setChannel(size_t channel) override;

private:
  void updateModulationParams();

  void transmitAwaitingData();
  void beginReceive();
  void receiveAwaitingData();
  void beginCad();

  bool eventsContain(RadioEvent event);
};

} // namespace VCTR::network::datalink

#endif