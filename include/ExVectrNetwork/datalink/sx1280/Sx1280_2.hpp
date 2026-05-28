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

#include "Sx1280_Settings.hpp"

#include "sx12xxAL/src/SX128XLT.h"

namespace VCTR::network::datalink {

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
                           public Core::Scheduler::Task {
public:
  Datalink_SX1280_V2(SX128XLT &sx1280Driver);

  // --- Getters ---------------------------------------------------------------
  int16_t lastPacketRSSI() const { return receivedDataRSSI; }
  int16_t lastPacketSNR() const { return receivedDataSNR; }

  // --- RadioI / DatalinkI overrides ------------------------------------------
  size_t getMaxPacketSize() const override;
  bool isChannelBlocked() const override;
  bool transmitDataframe(const DataPacket &dataframe) override;
  size_t getNumChannels() const override;
  size_t getCurrentChannel() const override;
  void setChannel(size_t channel) override;
  void setStartReceive(bool rxEnabled) override;
  void setEnableTxRx(bool enable) override;
  void setEnableAutoRx(bool enableAutoRx) override;

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
   * @param length  On-air byte count (including the 1-byte length prefix for
   *                Limited mode).
   */
  void setFixedPacketLength(uint8_t length);

  /// Sets the dB the external PA adds to the output power.
  void setPAdbm(uint8_t paDbm);

  uint16_t getRemainIrqFlags() const { return irqStatusRemain; }
  void clearRemainIrqFlags() { irqStatusRemain = 0; }

  SX128XLT &getRadio() { return lora; }

  void addTransmitFinishedHandler(std::function<void()> handler);

  // --- Interrupt Notify ------------------------------------------------------
  void notifyDio1Irq(int64_t timestamp, bool force = false);
  void fetchIrqFlags();

  // --- Task overrides --------------------------------------------------------
  void taskInit() override;
  void taskThread() override;
  void taskCheck() override;

  // --- Debug Stuff --------------------------------------------------------
  int getState() const { return (int)state; }
  int64_t getLastRunTime() const { return lastRun; }

private:
  // --- Constants -------------------------------------------------------------
  static constexpr size_t kMaxFrameLength = 128;

  static constexpr uint8_t kNumChannels = 20;
  static constexpr uint32_t kMinFreq = 2425000000UL;
  static constexpr uint32_t kMaxFreq = 2475000000UL;
  static constexpr uint32_t kChannelSpacing =
      (kMaxFreq - kMinFreq) / (kNumChannels - 1);

  static uint8_t moduleCount;

  // --- Radio states ----------------------------------------------------------
  enum class State { Sleep, Idle, IdleReceive, Receiving, Transmitting };

  // --- Hardware --------------------------------------------------------------
  SX128XLT &lora;

  // --- State -----------------------------------------------------------------
  State state = State::Sleep;
  bool txRxEnabled = false;   // Allow rx and tx to occur
  bool autoRxEnabled = true;  // Auto apply mod param changes and re enter rx.
  bool rxStartedFlag = false; // Trigger mod params to apply and then enter rx.
  bool leaveRxFlag = false;   // Stop idle receive if true
  bool acceptThisPacket = false;

  // --- IRQ Flags--------------------------------------------
  int64_t irqTrigTimestamp = 0;
  bool preambleDetected = false;
  bool headerValid = false;
  bool headerError = false;
  bool crcError = false;
  bool rxDone = false;
  bool txDone = false;
  bool rxTxTimeout = false;

  // --- Timestamps ------------------------------------------------
  int64_t txStartTimestamp = 0;
  int64_t txDoneTimestamp = 0;
  int64_t rxStartTimestamp = 0;
  int64_t rxDoneTimestamp = 0;
  int64_t rxIdleStartTimestamp = 0;
  int64_t lastRxSuccessTime = 0;
  // int64_t

  // --- Timing ------------------------------------------------
  // Software safety timeout — the radio now uses continuous RX (no HW
  // timeout), so this is only a fallback in case of stuck states.
  int64_t rxActiveTimeout = 50 * Core::MILLISECONDS;
  int64_t txActiveTimeout = 15 * Core::MILLISECONDS;
  // Earliest time before scheduled TX when the state machine may prepare/start
  // TX.
  int64_t txPrepareLeadTime = 3 * Core::MILLISECONDS;

  // --- TX pending data ----------------------------------
  uint8_t txBuffer[kMaxFrameLength];
  size_t txPendingSize = 0;
  size_t sxTxPendingSize = 0;
  int64_t pendingTxTime = 0;
  int64_t txScheduledTime = 0;

  // --- Last-receive stats ----------------------------------------------------
  int16_t receivedDataRSSI = 0;
  int16_t receivedDataSNR = 0;

  // --- TX power settings -----------------------------------------------------
  int8_t txPower = 0;
  int8_t lastTxPower = 0;
  int8_t maxTxPower = 20;
  uint8_t paGain = 0;

  // --- Modulation / frequency config -----------------------------------------
  bool modParamsChanged = true;
  bool freqChanged = true;
  SX1280_SF spreadingFactor = SX1280_SF::SF_8;
  SX1280_BW bandwidth = SX1280_BW::BW_800KHz;
  SX1280_CR codingRate = SX1280_CR::LI_4_8;
  uint32_t freq_hz = kMinFreq;

  uint16_t irqStatusRemain = 0;

  // --- Packet mode -----------------------------------------------------------
  bool packetParamsChanged = true;
  SX1280_PacketMode packetMode = SX1280_PacketMode::Dynamic;
  uint8_t fixedPacketLength = kMaxFrameLength;

  // --- Channel ---------------------------------------------------------------
  uint8_t currentChannel = 0;

  // --- Debug -----------------------------------------------------------------
  uint8_t moduleId = 0;
  int64_t lastTxPrint = 0;
  int64_t lastRxPrint = 0;
  int64_t lastRun = 0;

  // --- Handlers --------------------------------------------------------------
  Core::HandlerGroup<> transmitFinishedHandler;

  // --- Private helpers -------------------------------------------------------

  bool isActivelyReceiving() const;
  bool receiveFlagTrig() const;
  bool isTxReady() const;

  void clearReceiveFlags();
  void clearAllIrqFlags();

  void applyLoraParams();

  void applyFreqChange();

  void updateModParams();

  /**
   * Prepares the radio for transmission by placing data onto module and setting
   * parameters. Call startTx() to actually start transmission after this. Use
   * txStart to specify exactly when the transmission should start begin.
   */
  void prepareTx(const uint8_t *data, size_t size, int64_t txStart = 0);

  /**
   * Starts the transmission the that been prepared by prepareTx().
   * Will not wait till the txStart.
   */
  void startTx();

  /**
   * Puts the radio into RX mode.
   */
  void startIdleRx();

  /**
   * Puts the radio into active receiving mode
   */
  void startActiveRx();

  /**
   * Puts the radio into idle mode.
   */
  void startIdle();

  /**
   * Gets the data received from the radio and calls handlers.
   */
  void retrieveRxData();

  // --- State machine helpers ------------------------------------

  void updateReceiveState();

  void updateTransmitState();

  void updateIdleRxState();

  void updateIdleState();

  /**
   * Hardware-reset the SX1280 and re-apply all configuration.
   * Used as a last-resort recovery if the radio becomes stuck.
   */
  void fullReconfigure();
};

} // namespace VCTR::network::datalink

#endif