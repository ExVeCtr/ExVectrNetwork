#include <cmath>
#include <cstdarg>
#include <cstdio>

#include <Arduino.h>

#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/datalink/DatalinkI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"
#include "ExVectrNetwork/datalink/sx1280/sx12xxAL/src/SX128XLT.h"

// #define SX1280_DEBUG

namespace VCTR::network::datalink {

// ---------------------------------------------------------------------------
// Static
// ---------------------------------------------------------------------------

uint8_t Datalink_SX1280_V2::moduleCount = 0;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

Datalink_SX1280_V2::Datalink_SX1280_V2(SX128XLT &sx1280Driver)
    : Task_Periodic("Datalink_SX1280_V2", 100 * Core::MILLISECONDS),
      lora(sx1280Driver) {
  moduleId = moduleCount++;
  Core::getSystemScheduler().addTask(*this);
  setPriority(1000);
}

// ---------------------------------------------------------------------------
// DatalinkI / RadioI overrides
// ---------------------------------------------------------------------------

size_t Datalink_SX1280_V2::getMaxPacketSize() const { return kMaxFrameLength; }

bool Datalink_SX1280_V2::isChannelBlocked() const {
  bool blocked = txScheduledTime != 0 || txPendingSize > 0 || !enableTxRx;

  if (blocked) {
    // Serial.printf("[SX1280 %d] %.3f Channel is blocked by %s%s%s\n",
    // moduleId,
    //               (double)Core::NOW() / Core::SECONDS,
    //               txScheduledTime != 0 ? "scheduled TX " : "",
    //               txPendingSize > 0 ? "pending TX " : "",
    //               !enableTxRx ? "TX disabled" : "");
  }
  return blocked;
}

bool Datalink_SX1280_V2::transmitDataframe(const DataPacket &dataframe) {
  if (isChannelBlocked()) {
    return false;
  }

  auto len = dataframe.payload.size();
  if (len > kMaxFrameLength) {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] Frame too large (%d > %d)\n", moduleId, (int)len,
                  (int)kMaxFrameLength);
#endif
    return false;
  }

  auto scheduledTxTime =
      dataframe.timestamp == 0 ? Core::NOW() : dataframe.timestamp;

  if (sxTxPendingSize == 0 &&
      (state == State::Idle || state == State::IdleReceive)) {
    prepareTx(dataframe.payload.getPtr(), dataframe.payload.size(),
              scheduledTxTime);
  } else {
    memcpy(txBuffer, dataframe.payload.getPtr(), len);
    txPendingSize = len;
    txScheduledTime = scheduledTxTime;
  }

  return true;
}

size_t Datalink_SX1280_V2::getNumChannels() const { return kNumChannels; }
size_t Datalink_SX1280_V2::getCurrentChannel() const { return currentChannel; }

void Datalink_SX1280_V2::setChannel(size_t channel) {
  channel = channel % kNumChannels;
  currentChannel = channel;
  freq_hz = kMinFreq + channel * kChannelSpacing;

  freqChanged = true;
}

// ---------------------------------------------------------------------------
// Configuration setters
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::setFrequency(uint32_t freq_hz) {
  freqChanged = true;
  freq_hz = freq_hz;
}

void Datalink_SX1280_V2::setSpreadingFactor(SX1280_SF sf) {
  modParamsChanged = true;
  spreadingFactor = sf;
}

void Datalink_SX1280_V2::setBandwidth(SX1280_BW bw) {
  modParamsChanged = true;
  bandwidth = bw;
}

void Datalink_SX1280_V2::setCodingRate(SX1280_CR cr) {
  modParamsChanged = true;
  codingRate = cr;
}

void Datalink_SX1280_V2::setTxPower(int8_t power) { txPower = power; }

void Datalink_SX1280_V2::setTxMaxPower(int8_t maxTxPower) {
  maxTxPower = maxTxPower;
}

void Datalink_SX1280_V2::setPAdbm(uint8_t paDbm) { paGain = paDbm; }

void Datalink_SX1280_V2::setEnableTxRx(bool enable) { enableTxRx = enable; }

void Datalink_SX1280_V2::addTransmitFinishedHandler(
    std::function<void()> handler) {
  transmitFinishedHandler.addHandler(handler);
}

void Datalink_SX1280_V2::notifyDio1Irq(int64_t timestamp, bool force) {
  if (force || irqTrigTimestamp == 0) {
    irqTrigTimestamp = timestamp;
  }
}

void Datalink_SX1280_V2::taskInit() {
  if (!lora.checkDevice()) {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] Device check FAILED\n", moduleId);
#endif
    setInitialised(false);
    this->setRelease(Core::NOW() + 1 * Core::SECONDS);
    return;
  }

#ifdef SX1280_DEBUG
  Serial.printf("[SX1280 %d] Device found, configuring\n", moduleId);
#endif

  lora.setupLoRa(freq_hz, 0, spreadingFactor, bandwidth, codingRate, false);
  lora.setHighSensitivity();
  lora.setBufferBaseAddress(128, 0);
  lora.setDioIrqParams(IRQ_RADIO_ALL,
                       (IRQ_TX_DONE /* | IRQ_PREAMBLE_DETECTED*/ | IRQ_RX_DONE |
                        IRQ_RX_TX_TIMEOUT),
                       0, 0);

  // Mark as applied so enterRx() doesn't redo them immediately.
  modParamsChanged = false;
  freqChanged = false;

  state = State::Idle;
}

void Datalink_SX1280_V2::taskCheck() {

  bool paramsUpdate = (modParamsChanged || freqChanged) &&
                      (state == State::Idle || state == State::IdleReceive);
  bool irqTrig = irqTrigTimestamp != 0;
  bool rxReady = receiveFlagTrig();
  bool txReady =
      isTxReady() && (state == State::Idle || state == State::IdleReceive);
  bool txPrepare =
      txPendingSize > 0 && sxTxPendingSize == 0 && (state != State::Receiving);
  // Only leave idle if there's no pending TX waiting for its scheduled time.
  bool txWaiting = sxTxPendingSize > 0 && !isTxReady();
  bool leaveIdle = state == State::Idle && !txWaiting;

  if (paramsUpdate || irqTrig || rxReady || txReady || txPrepare || leaveIdle ||
      txDone) {
    setDeadline(Core::NOW());
  } else if (txWaiting) {
    // Wake up at the TX scheduled time rather than busy-looping.
    setDeadline(txScheduledTime);
  }
}

void Datalink_SX1280_V2::taskThread() {

  auto threadStartTime = Core::NOW();

  fetchIrqFlags();

  // Serial.printf("%.3f State: %d\n", Core::NOWSeconds(), state);

  switch (state) {
    // In receive mode but not actively receiving a packet. Waiting for
    // preamble.
  case State::IdleReceive:
    updateIdleRxState();
    break;

  // Actively receiving a packet. Waiting for RX_DONE or timeout.
  case State::Receiving: {
    updateReceiveState();
    break;
  }

  // Actively transmitting a packet. Waiting for TX_DONE or timeout.
  case State::Transmitting: {
    updateTransmitState();
    break;
  }

  case State::Idle:
  case State::Sleep:
  default: {
    updateIdleState();
    break;
  }
  }

  if (txPendingSize > 0 && sxTxPendingSize == 0 &&
      (state == State::Idle || state == State::IdleReceive)) {
    prepareTx(txBuffer, txPendingSize, txScheduledTime);
  }

  irqTrigTimestamp = 0; // reset for next DIO1 event
  // clearAllIrqFlags();
}

void Datalink_SX1280_V2::fetchIrqFlags() {

  int64_t now = Core::NOW();

  // Read dio1 if its not been done yet.
  if (irqTrigTimestamp == 0) {
    irqTrigTimestamp = now;
  }

  auto irqStatus = lora.readIrqStatus();
  auto irqStatusSeen = 0;

  if (irqStatus & IRQ_PREAMBLE_DETECTED) {
    irqStatusSeen |= IRQ_PREAMBLE_DETECTED;
    preambleDetected = true;
    rxStartTimestamp = irqTrigTimestamp;
  }

  if (irqStatus & IRQ_HEADER_VALID) {
    irqStatusSeen |= IRQ_HEADER_VALID;
    headerValid = true;
  }

  if (irqStatus & IRQ_HEADER_ERROR) {
    irqStatusSeen |= IRQ_HEADER_ERROR;
    headerError = true;
  }

  if (irqStatus & IRQ_CRC_ERROR) {
    irqStatusSeen |= IRQ_CRC_ERROR;
    crcError = true;
  }

  if (irqStatus & IRQ_RX_DONE) {
    irqStatusSeen |= IRQ_RX_DONE;
    rxDone = true;
    rxDoneTimestamp = irqTrigTimestamp;
  }

  if (irqStatus & IRQ_TX_DONE) {
    irqStatusSeen |= IRQ_TX_DONE;
    txDone = true;
    txDoneTimestamp = irqTrigTimestamp;
  }

  if (irqStatus & IRQ_RX_TX_TIMEOUT) {
    irqStatusSeen |= IRQ_RX_TX_TIMEOUT;
    rxTxTimeout = true;
  }

  irqStatusRemain |= irqStatus & ~irqStatusSeen;

  irqTrigTimestamp = 0;

  if (irqStatus) {
    lora.clearIrqStatus(irqStatus);
  }
}

bool Datalink_SX1280_V2::isActivelyReceiving() const {
  return state == State::Receiving || receiveFlagTrig();
}

bool Datalink_SX1280_V2::receiveFlagTrig() const {
  return rxDone /* || preambleDetected*/ || crcError || headerValid ||
         headerError;
}

bool Datalink_SX1280_V2::isTxReady() const {
  return sxTxPendingSize > 0 && Core::NOW() > txScheduledTime;
}

void Datalink_SX1280_V2::clearReceiveFlags() {
  headerValid = false;
  headerError = false;
  crcError = false;
  rxDone = false;
  preambleDetected = false;
  rxTxTimeout = false;
}

void Datalink_SX1280_V2::clearAllIrqFlags() {
  clearReceiveFlags();
  txDone = false;
  rxTxTimeout = false;
}

void Datalink_SX1280_V2::applyLoraParams() {
  lora.setMode(MODE_STDBY_XOSC);
  lora.setModulationParams(spreadingFactor, bandwidth, codingRate);
  modParamsChanged = false;
  state = State::Idle;
}

void Datalink_SX1280_V2::applyFreqChange() {
  lora.setMode(MODE_STDBY_XOSC);
  lora.setRfFrequency(freq_hz, 0);
  freqChanged = false;
  state = State::Idle;
  // Serial.printf("[SX1280 %d] Frequency set to %lu Hz\n", moduleId, freq_hz);
}

void Datalink_SX1280_V2::prepareTx(const uint8_t *data, size_t size,
                                   int64_t txStart) {
  txScheduledTime = txStart == 0 ? Core::NOW() : txStart;
  sxTxPendingSize = size;

  // Clear stale receive flags before putting the radio in STDBY.
  // startWriteSXBuffer calls setMode(MODE_STDBY_RC) which aborts any ongoing
  // RX. Without clearing these flags, the state machine would incorrectly
  // re-enter Receiving state and block the pending TX.
  clearReceiveFlags();
  state = State::Idle;

  lora.startWriteSXBuffer(128);
  lora.writeBufferRaw(data, sxTxPendingSize);
  lora.endWriteSXBuffer();
  lora.setPayloadLength(static_cast<uint8_t>(sxTxPendingSize));

  // Compute effective power.
  int8_t power = txPower;
  if (power > maxTxPower)
    power = maxTxPower;
  power = power - (int8_t)paGain;

  if (lastTxPower != power) {
    lora.setTxParams(power, RAMP_TIME);
    lastTxPower = power;
  }

  txPendingSize = 0;

  // Serial.printf("%.4f, Tx Prepared\n", Core::NOWSeconds());
}

void Datalink_SX1280_V2::startTx() {
  txScheduledTime = 0;
  txStartTimestamp = Core::NOW();
  // lora.setBufferBaseAddress(128, 0);
  lora.setTx(0);
  state = State::Transmitting;
  // auto end = Core::NOW();
  // Serial.printf("%.1f, Tx Started. dT: %.1f, since last: %.1f\n",
  //               (double)txStartTimestamp / Core::MILLISECONDS,
  //               (double)(end - txStartTimestamp) / Core::MILLISECONDS,
  //               (double)(txStartTimestamp - lastTxPrint) /
  //               Core::MILLISECONDS);
  // lastTxPrint = txStartTimestamp;
}

void Datalink_SX1280_V2::startIdleRx() {
  // lora.setBufferBaseAddress(128, 0);
  lora.setRx(rxActiveTimeout / Core::MILLISECONDS);
  state = State::IdleReceive;
  rxIdleStartTimestamp = Core::NOW();
}

void Datalink_SX1280_V2::startActiveRx() {
  // irqStatusRemain |=
  //     (crcError ? IRQ_CRC_ERROR : 0) | (headerError ? IRQ_HEADER_ERROR : 0) |
  //     (preambleDetected ? IRQ_PREAMBLE_DETECTED : 0) |
  //     (headerValid ? IRQ_HEADER_VALID : 0) | (rxDone ? IRQ_RX_DONE : 0);
  if (rxStartTimestamp == 0) {
    rxStartTimestamp = Core::NOW();
  }
  state = State::Receiving;
}

void Datalink_SX1280_V2::retrieveRxData() {

  // lora.setMode(MODE_STDBY_XOSC);

  receivedDataRSSI = lora.readPacketRSSI();
  receivedDataSNR = lora.readPacketSNR();

  auto len = lora.readRXPacketL();

  uint8_t buffer[len];
  lora.startReadSXBuffer(0);
  lora.readBuffer(buffer, len);
  lora.endReadSXBuffer();

  auto tOA = lora.getLoRaTimeOnAirMs(len) * Core::MILLISECONDS;
  auto packetRecvStartTime = rxDoneTimestamp - tOA;
  DataPacket packet;
  packet.timestamp = packetRecvStartTime;
  packet.payload.setSize(len);
  memcpy(packet.payload.getPtr(), buffer, len);

  // Serial.printf("[SX1280 %d] Packet received. RxDone: %.1f, ToA: %.1f ms, Est
  // "
  //               "start: %.1f, RSSI: %d, SNR: %d, Len: %d\n",
  //               moduleId, Core::NOWSeconds() * 1000,
  //               (double)tOA / Core::MILLISECONDS,
  //               (double)packetRecvStartTime / Core::MILLISECONDS,
  //               receivedDataRSSI, receivedDataSNR, (int)len);

  receiveHandlers_.callHandlers(packet);
}

void Datalink_SX1280_V2::updateReceiveState() {

  if (rxDone) {
    if (!crcError && headerValid && enableTxRx) {
      retrieveRxData();
    }

    rxDoneTimestamp = rxStartTimestamp = 0;
    state = State::Idle;
    clearReceiveFlags();
  } else if (Core::NOW() - rxStartTimestamp > rxActiveTimeout) {
    // Serial.printf("%.4f, Rx Timeout\n", Core::NOWSeconds());
    rxDoneTimestamp = rxStartTimestamp = 0;
    state = State::Idle;
    clearReceiveFlags();
  } else if (crcError || headerError || rxTxTimeout || !enableTxRx) {
    rxDoneTimestamp = rxStartTimestamp = 0;
    state = State::Idle;
    clearReceiveFlags();
  }

  // clearReceiveFlags();
}

void Datalink_SX1280_V2::updateTransmitState() {
  if (txDone || Core::NOW() - txStartTimestamp > txActiveTimeout) {
    transmitFinishedHandler.callHandlers();
    sxTxPendingSize = 0;
    txStartTimestamp = 0;
    txDoneTimestamp = 0;
    state = State::Idle;
    rxTxTimeout = false;
    txDone = false;
  }
}

void Datalink_SX1280_V2::updateIdleRxState() {

  if (isActivelyReceiving()) {
    startActiveRx();
    updateReceiveState();
  } else if (rxTxTimeout || (rxIdleStartTimestamp > 0 &&
                             Core::NOW() - rxIdleStartTimestamp >
                                 rxActiveTimeout + 500 * Core::MILLISECONDS)) {
    // RX timeout: the hardware has returned to STDBY.
    // Transition to Idle so the radio re-enters RX on the next tick.
    rxTxTimeout = false;
    rxIdleStartTimestamp = 0;
    state = State::Idle;
  } else if (freqChanged) {
    applyFreqChange();
  } else if (modParamsChanged) {
    applyLoraParams();
  } else if (isTxReady()) {
    startTx();
  }
}

void Datalink_SX1280_V2::updateIdleState() {

  if (freqChanged) {
    applyFreqChange();
  } else if (modParamsChanged) {
    applyLoraParams();
  } else if (isTxReady()) {
    startTx();
  } else {
    startIdleRx();
  }
}

} // namespace VCTR::network::datalink