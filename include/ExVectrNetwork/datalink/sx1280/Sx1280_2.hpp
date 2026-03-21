#ifndef EXVECTRNETWORK_DATALINKSX1280_H_
#define EXVECTRNETWORK_DATALINKSX1280_H_

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
 * @brief Datalink layer for the SX1280 LoRa transceiver.
 *
 * Design:
 *  - The radio sits in continuous RX (timeout = 0xFFFF) at all times.
 *  - When a preamble is detected the timestamp is recorded (for FHSS sync).
 *  - When IRQ_RX_DONE fires the packet is read from the buffer and dispatched.
 *  - To transmit: the radio is moved to STDBY, the packet is loaded, TX is
 *    started, and on TX_DONE the radio goes straight back to continuous RX.
 *  - No CAD is used.
 *  - Frequency / modulation parameter changes are applied by restarting RX.
 */
class Datalink_SX1280_V2 : public VCTR::network::datalink::RadioI,
                           public VCTR::Core::Task_Periodic {
public:
  Datalink_SX1280_V2(SX128XLT &sx1280Driver);

  // --- Getters ---------------------------------------------------------------
  int16_t lastPacketRSSI() const { return receivedDataRSSI_; }
  int16_t lastPacketSNR() const { return receivedDataSNR_; }

  /// Timestamp (Core::NOW() units) of when the last preamble was detected.
  /// Useful for FHSS synchronisation in higher layers.
  int64_t lastReceiveStartTimestamp() const { return receiveStartTimestamp_; }

  // --- RadioI / DatalinkI overrides ------------------------------------------
  size_t getMaxPacketSize() const override;
  bool isChannelBlocked() const override;
  bool transmitDataframe(const DataPacket &dataframe) override;
  size_t getNumChannels() const override;
  size_t getCurrentChannel() const override;
  void setChannel(size_t channel) override;

  // --- Configuration ---------------------------------------------------------
  void setFrequency(uint32_t freq_hz);
  void setSpreadingFactor(SX1280_SF sf);
  void setBandwidth(SX1280_BW bw);
  void setCodingRate(SX1280_CR cr);
  void setTxPower(int8_t power);
  void setTxMaxPower(int8_t maxTxPower);

  /// Sets the dB the external PA adds to the output power.
  void setPAdbm(uint8_t paDbm);

  /**
   * @brief Enable or disable all radio activity.
   * When disabled the radio is placed in STDBY.
   */
  void setEnableTxRx(bool enable);

  void addTransmitFinishedHandler(std::function<void()> handler);

  // --- Task overrides --------------------------------------------------------
  void taskInit() override;
  void taskThread() override;
  void taskCheck() override;

private:
  // --- Constants -------------------------------------------------------------
  static constexpr size_t kMaxFrameLength = 200;

  static constexpr uint8_t kNumChannels = 10;
  static constexpr uint32_t kMinFreq = 2410000000UL;
  static constexpr uint32_t kMaxFreq = 2470000000UL;
  static constexpr uint32_t kChannelSpacing =
      (kMaxFreq - kMinFreq) / (kNumChannels - 1);

  static uint8_t moduleCount_;

  // --- Radio states ----------------------------------------------------------
  enum class State { Idle, Receiving, Transmitting };

  // --- Hardware --------------------------------------------------------------
  SX128XLT &lora_;

  // --- State -----------------------------------------------------------------
  State state_ = State::Idle;
  bool enableTxRx_ =
      false; /// True from preamble-detected until RX_DONE / error / enterRx().
  /// Exposed via isChannelBlocked() so higher layers can detect active RX.
  bool preambleActive_ = false;
  // --- Timestamps (nanoseconds, Core::NOW() units) ---------------------------
  int64_t receiveStartTimestamp_ = 0; ///< Preamble-detect time for FHSS.
  int64_t transmitStartTime_ = 0;     ///< When TX was started (for timeout).
  int64_t lastIrqActivityTime_ = 0;   ///< Last time any IRQ bits were read.
  int64_t irqTrigTimestamp_ = 0;      ///< When we first saw DIO1 hig
  int64_t recvFinishTimestamp_ = 0; ///< When RX_DONE was detected (for stats).
  int64_t recvStartTime_ = 0; ///< When we entered receive mode (for stats).
  int64_t threadStartTime_ = 0; ///< When taskThread() was entered (for stats).

  // --- IRQ accumulator -------------------------------------------------------
  /// Hardware IRQ bits are cleared immediately after reading (to prevent DIO1
  /// staying high and causing a tight poll loop). The bits are OR-ed into this
  /// accumulator so the state machine can still see the full picture across
  /// multiple reads within one receive/transmit cycle.  Reset on every state
  /// transition (enterRx / startTransmit).
  uint16_t irqAccum_ = 0;

  // --- TX pending data -------------------------------------------------------
  size_t txPendingSize_ = 0; ///< >0 means data is in the SX buffer.

  // --- Last-receive stats ----------------------------------------------------
  int16_t receivedDataRSSI_ = 0;
  int16_t receivedDataSNR_ = 0;

  // --- TX power settings -----------------------------------------------------
  int8_t txPower_ = 0;
  int8_t maxTxPower_ = 12;
  uint8_t paDbm_ = 0;

  // --- Modulation / frequency config -----------------------------------------
  bool modParamsChanged_ = true;
  bool freqChanged_ = true;
  SX1280_SF spreadingFactor_ = SX1280_SF::SF_8;
  SX1280_BW bandwidth_ = SX1280_BW::BW_800KHz;
  SX1280_CR codingRate_ = SX1280_CR::LI_4_8;
  uint32_t freq_hz_ = kMinFreq;

  SX1280_SF spreadingFactorLast_ = SX1280_SF::SF_8;
  SX1280_BW bandwidthLast_ = SX1280_BW::BW_800KHz;
  SX1280_CR codingRateLast_ = SX1280_CR::LI_4_8;
  uint32_t freqLast_hz_ = kMinFreq;

  // --- Channel ---------------------------------------------------------------
  uint8_t currentChannel_ = 0;

  // --- Debug -----------------------------------------------------------------
  uint8_t moduleId_ = 0;

  // --- Handlers --------------------------------------------------------------
  Core::HandlerGroup<> transmitFinishedHandler_;

  // --- Private helpers -------------------------------------------------------

  /// Calculate the on-air time (in nanoseconds) of a LoRa packet given the
  /// current modulation parameters and the payload size in bytes.
  int64_t calcPacketAirtime(size_t payloadBytes) const;

  /// Apply any pending modulation / frequency changes while in STDBY.
  void applyParamChanges();

  /// Enter continuous receive mode (timeout = 0xFFFF).
  void enterRx();

  /// Read a completed packet from the radio buffer and dispatch it.
  void handleReceivedPacket();

  /// Start transmitting the data already in the SX buffer.
  void startTransmit();
};

} // namespace VCTR::network::datalink

#endif