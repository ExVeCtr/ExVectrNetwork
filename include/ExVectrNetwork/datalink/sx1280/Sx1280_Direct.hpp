#ifndef EXVECTRNETWORK_SX1280_DIRECT_HPP_
#define EXVECTRNETWORK_SX1280_DIRECT_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrHAL/digital_io.hpp"
#include "ExVectrHAL/pin_gpio.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/physical/HasChannels.hpp"

#include "Sx1280_Settings.hpp"

#include "sx12xxAL/src/SX128XLT.h"

#include <stddef.h>
#include <stdint.h>

namespace VCTR::network::datalink {

/**
 * @brief Thin staged-control wrapper for the SX1280 LoRa transceiver.
 *
 * Call configureRadio() once to initialise the radio, then use the setters to
 * stage changes and optionally a TX packet. push() applies staged changes while
 * putting the radio into standby. startRx() and startTx() only issue the
 * timing-critical command that begins the operation. pull() polls IRQ state and
 * harvests any completed RX packet.
 */
class Sx1280_Direct : public VCTR::network::physical::HasChannels {
public:
  Sx1280_Direct(SX128XLT &sx1280Driver);

  bool configureRadio();

  // --- Receiving -----------------------------------------
  void startRx(int64_t timeout);
  int16_t getPacketRSSI() const;
  int16_t getPacketSNR() const;
  network::DataPacket getRxPacket() const;
  uint32_t getRxPacketCount() const;

  // --- Transmitting ---------------------------------------------
  bool setupTxPacket(const network::DataPacket &packet);
  void startTx();
  uint32_t getTxPacketCount() const;

  uint8_t *getTxBufferPtr();
  uint8_t getTxBufferSize() const;

  // --- HasChannels overrides -------------------------------------------------
  size_t getNumChannels() const override;
  size_t getCurrentChannel() const override;
  void setChannel(size_t channel) override;

  // --- Configuration ---------------------------------------------------------
  void setFrequency(uint32_t newFreqHz);
  void setSpreadingFactor(SX1280_SF sf);
  void setBandwidth(SX1280_BW bw);
  void setCodingRate(SX1280_CR cr);
  void setTxPower(int8_t power);
  uint8_t getTxPower() const { return txPower; }
  void setTxMaxPower(int8_t maxTxPower);

  /**
   * @brief Set the packet framing mode.  Must be called before taskInit().
   */
  void setPacketMode(SX1280_PacketMode mode);

  /**
   * @brief Set the fixed on-air packet length for Limited / Fixed modes.
   *        Must be called before taskInit().
   * @param length  User payload length for Limited mode and exact payload
   *                length for Fixed mode.
   */
  void setFixedPacketLength(uint8_t length);

  /// Sets the dB the external PA adds to the output power.
  void setPAdbm(uint8_t paDbm);

  /**
   * @brief Apply any pending configuration changes (frequency, modulation,
   * packet), also places the to tx packet into the SX1280 buffer if
   * setupTxPacket was called.
   */
  void push(bool keepOscRunning = false);
  /**
   * @brief Polls the radio for IRQ flags and rx packet.
   */
  void pull();

  // --- Interrupt and flags ------------------------------
  void notifyDio1Irq(int64_t timestamp, bool force = false);
  void fetchIrqFlags();

private:
  // --- Constants -------------------------------------------------------------
  static constexpr size_t kMaxFrameLength = 128;
  static constexpr uint8_t kTxBufferAddress = 128;
  static constexpr uint8_t kRxBufferAddress = 0;
  static constexpr uint16_t kRadioTimeoutMax = 0xFFFF;

  static constexpr uint8_t kNumChannels = 20;
  static constexpr uint32_t kMinFreq = 2425000000UL;
  static constexpr uint32_t kMaxFreq = 2475000000UL;
  static constexpr uint32_t kChannelSpacing =
      (kMaxFreq - kMinFreq) / (kNumChannels - 1);

  // --- Radio states ----------------------------------------------------------
  enum class State { Sleep, Idle, IdleReceive, Receiving, Transmitting };

  // --- Hardware --------------------------------------------------------------
  SX128XLT &lora;

  // --- State -----------------------------------------------------------------
  State state = State::Sleep;
  bool modParamsChanged = true;
  bool freqChanged = true;
  bool txPacketPending = false;
  bool txPacketLoaded = false;

  // --- IRQ Flags--------------------------------------------
  int64_t irqTrigTimestamp = 0;
  bool preambleDetected = false;
  bool headerValid = false;
  bool headerError = false;
  bool crcError = false;
  bool rxDone = false;
  bool txDone = false;
  bool rxTxTimeout = false;
  int64_t lastRxTimestamp = 0;
  uint16_t irqStatusRemain = 0;

  // --- Last-receive stats ----------------------------------------------------
  int16_t receivedDataRSSI = 0;
  int16_t receivedDataSNR = 0;
  network::DataPacket lastRxPacket;
  uint32_t rxPacketCount = 0;
  uint32_t txPacketCount = 0;

  // --- TX power settings -----------------------------------------------------
  int8_t txPower = 0;
  int8_t maxTxPower = 20;
  uint8_t paGain = 0;

  // --- Modulation / frequency config -----------------------------------------
  SX1280_SF spreadingFactor = SX1280_SF::SF_8;
  SX1280_BW bandwidth = SX1280_BW::BW_800KHz;
  SX1280_CR codingRate = SX1280_CR::LI_4_8;
  uint32_t freq_hz = kMinFreq;

  // --- Packet mode -----------------------------------------------------------
  bool packetParamsChanged = true;
  SX1280_PacketMode packetMode = SX1280_PacketMode::Dynamic;
  uint8_t fixedPacketLength = kMaxFrameLength;
  uint8_t txBuffer[kMaxFrameLength] = {0};
  size_t txPendingSize = 0;
  size_t txLoadedSize = 0;

  // --- Channel ---------------------------------------------------------------
  uint8_t currentChannel = 0;

  // --- Private helpers -------------------------------------------------------

  size_t getMaxPayloadSize() const;
  int8_t getAppliedTxPower() const;
  uint16_t clampRadioTimeout(int64_t timeout) const;
  void clearIrqFlags();
  void applyPacketParams();
  void prepareTxPacket(const uint8_t *data, size_t size);
  void readCompletedPacket();
};

} // namespace VCTR::network::datalink

#endif