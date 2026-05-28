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
    : Core::Scheduler::Task("Datalink_SX1280_V2"), lora(sx1280Driver) {
  moduleId = moduleCount++;
  Core::getSystemScheduler().addTask(*this);
  setPriority(1000);
}

// ---------------------------------------------------------------------------
// DatalinkI / RadioI overrides
// ---------------------------------------------------------------------------

size_t Datalink_SX1280_V2::getMaxPacketSize() const {
  switch (packetMode) {
  case SX1280_PacketMode::Limited:
    return fixedPacketLength;
  case SX1280_PacketMode::Fixed:
    return fixedPacketLength;
  case SX1280_PacketMode::Dynamic:
  default:
    return kMaxFrameLength;
  }
}

bool Datalink_SX1280_V2::isChannelBlocked() const {
  bool blocked = !txRxEnabled || txPendingSize > 0;

  // if (blocked) {
  //   Serial.printf("Channel is blocked. txRxEnabled=%d, txPendingSize=%d\n",
  //                 (int)txRxEnabled, (int)txPendingSize);
  // }
  return blocked;
}

bool Datalink_SX1280_V2::transmitDataframe(const DataPacket &dataframe) {
  // Serial.printf("[SX1280 %d] Request to transmit packet of size %d bytes\n",
  //               moduleId, (int)dataframe.payload.size());
  if (isChannelBlocked()) {
    return false;
  }

  // Serial.printf("[SX1280 %d] Channel is clear for transmission\n", moduleId);

  auto len = dataframe.payload.size();
  if (len > getMaxPacketSize()) {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] Frame too large (%d > %d)\n", moduleId, (int)len,
                  (int)getMaxPacketSize());
#endif
    return false;
  }

  auto scheduledTxTime =
      dataframe.timestamp == 0 ? Core::NowNs() : dataframe.timestamp;

  if (sxTxPendingSize == 0 &&
      (state == State::Idle || state == State::IdleReceive)) {
    prepareTx(dataframe.payload.getPtr(), dataframe.payload.size(),
              scheduledTxTime);
  } else {
    memcpy(txBuffer, dataframe.payload.getPtr(), len);
    txPendingSize = len;
    pendingTxTime = scheduledTxTime;
  }

  return true;
}

size_t Datalink_SX1280_V2::getNumChannels() const { return kNumChannels; }
size_t Datalink_SX1280_V2::getCurrentChannel() const { return currentChannel; }

void Datalink_SX1280_V2::setChannel(size_t channel) {
  channel = channel % kNumChannels;
  uint32_t newFreq = kMinFreq + channel * kChannelSpacing;
  currentChannel = channel;
  if (newFreq != freq_hz) {
    freq_hz = newFreq;
    freqChanged = true;
  }
}

// ---------------------------------------------------------------------------
// Configuration setters
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::setFrequency(uint32_t newFreqHz) {
  if (newFreqHz != freq_hz) {
    freq_hz = newFreqHz;
    freqChanged = true;
  }
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
  this->maxTxPower = maxTxPower;
}

void Datalink_SX1280_V2::setPAdbm(uint8_t paDbm) { paGain = paDbm; }

void Datalink_SX1280_V2::setPacketMode(SX1280_PacketMode mode) {
  if (mode != packetMode) {
    packetMode = mode;
    packetParamsChanged = true;
  }
}

void Datalink_SX1280_V2::setFixedPacketLength(uint8_t length) {
  if (length != fixedPacketLength) {
    fixedPacketLength = length;
    packetParamsChanged = true;
  }
}

void Datalink_SX1280_V2::setStartReceive(bool enable) {
  rxStartedFlag = enable;
  leaveRxFlag = !enable;
  if (leaveRxFlag && state == State::IdleReceive) {
    startIdle();
    leaveRxFlag = false;
  } else if (rxStartedFlag) {
    if (state == State::Idle) {
      // Can enter RX immediately.
      rxStartedFlag = false;
      updateModParams();
      startIdleRx();
    } else if (state == State::IdleReceive) {
      rxStartedFlag = false;
      if (modParamsChanged || freqChanged || packetParamsChanged) {
        updateModParams();
        startIdleRx();
      }
    }
    // If state is Transmitting or Receiving, leave rxStartedFlag = true
    // so the state machine enters RX once the current operation finishes.
  }
}

void Datalink_SX1280_V2::setEnableTxRx(bool enable) {
  txRxEnabled = enable;
  // if (!txRxEnabled) {
  //   txPendingSize = 0;
  //   startIdle();
  // }
}

void Datalink_SX1280_V2::setEnableAutoRx(bool enable) {
  autoRxEnabled = enable;
}

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
    this->setRelease(Core::NowNs() + 1 * Core::SECONDS);
    return;
  }

#ifdef SX1280_DEBUG
  Serial.printf("[SX1280 %d] Device found, configuring\n", moduleId);
#endif

  lora.setupLoRa(freq_hz, 0, spreadingFactor, bandwidth, codingRate, false);
  lora.setHighSensitivity();
  lora.setBufferBaseAddress(128, 0);

  // Enable AutoFS: after RX/TX the radio goes to FS (frequency-synthesis)
  // mode instead of STDBY_RC.  This keeps the PLL locked and avoids the
  // ~200µs cold-start penalty on each RX/TX entry (ELRS pattern).
  lora.setAutoFS(true);

  // Override packet params for implicit header modes.
  if (packetMode == SX1280_PacketMode::Limited) {
    // OTA size = usable payload + 1-byte length prefix
    lora.setPacketParams(12, LORA_PACKET_FIXED_LENGTH, fixedPacketLength + 1,
                         LORA_CRC_ON, LORA_IQ_NORMAL);
  } else if (packetMode == SX1280_PacketMode::Fixed) {
    lora.setPacketParams(12, LORA_PACKET_FIXED_LENGTH, fixedPacketLength,
                         LORA_CRC_ON, LORA_IQ_NORMAL);
  } else {
    lora.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON,
                         LORA_IQ_NORMAL);
  }

  // DIO1 mask: only fire the pin ISR on TX_DONE and RX_DONE.
  // IRQ mask stays IRQ_RADIO_ALL so we can poll preamble/CRC/timeout
  // via SPI in fetchIrqFlags() without generating spurious DIO1 edges
  // every 15ms (the old timeout-driven ISR storm).
  lora.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE | IRQ_RX_DONE), 0, 0);

  // Mark as applied so enterRx() doesn't redo them immediately.
  modParamsChanged = false;
  freqChanged = false;
  packetParamsChanged = false;

  lastRxSuccessTime = Core::NowNs();

  startIdle();
}

void Datalink_SX1280_V2::taskCheck() {

  // bool paramsUpdate =
  //     (modParamsChanged || freqChanged || packetParamsChanged) &&
  //     (state == State::Idle || state == State::IdleReceive) &&
  //     (autoRxEnabled || rxStartedFlag);
  // bool irqTrig = irqTrigTimestamp != 0;
  // bool rxReady = receiveFlagTrig();
  // bool txReady =
  //     isTxReady() && (state == State::Idle || state == State::IdleReceive);
  // bool txPrepare =
  //     txPendingSize > 0 && sxTxPendingSize == 0 && (state !=
  //     State::Receiving);
  // bool startRx = rxStartedFlag && state == State::Idle;
  // bool stopIdleRx = leaveRxFlag && state == State::IdleReceive;

  // if (paramsUpdate || irqTrig || rxReady || txReady || txPrepare || startRx
  // ||
  //     stopIdleRx || txDone) {
  //   setDeadline(Core::NowNs());
  // }

  // If dio1 IRQ triggered, run immediately.
  if (irqTrigTimestamp) {
    setDeadline(Core::NowNs());
    // Serial.printf("[SX1280 %d] DIO1 IRQ triggered.\n", moduleId);
    return;
  }

  // If TX is queued for a future timestamp, schedule a wakeup at the
  // configured lead-time boundary so startTx() can wait only a short time.
  if (sxTxPendingSize > 0 && txScheduledTime != 0 &&
      state != State::Transmitting) {
    int64_t now = Core::NowNs();
    int64_t wakeTime = txScheduledTime - txPrepareLeadTime;
    if (wakeTime < now) {
      wakeTime = now;
    }
    setRelease(wakeTime);
    setDeadline(wakeTime);
  }

  bool txSendTrig =
      isTxReady() && (state == State::Idle || state == State::IdleReceive);
  bool idleToRxIdle = rxStartedFlag && state == State::Idle;

  bool txTimeoutTrig = state == State::Transmitting && txStartTimestamp != 0 &&
                       Core::NowNs() - txStartTimestamp > txActiveTimeout;

  bool txDoneTrig = txTimeoutTrig || txDone;

  // Software safety: if we've been in IdleReceive longer than
  // rxActiveTimeout with no DIO1 event, check for stuck radio.
  bool idleRxSafetyTrig =
      state == State::IdleReceive && rxIdleStartTimestamp != 0 &&
      Core::NowNs() - rxIdleStartTimestamp > rxActiveTimeout;
  bool rxActiveTrig =
      rxDone || preambleDetected || crcError || headerValid || headerError;

  bool txRxTimeoutTrig = rxTxTimeout;

  if (txSendTrig || idleToRxIdle || txDoneTrig || idleRxSafetyTrig ||
      rxActiveTrig || txRxTimeoutTrig) {
    // Serial.printf("[SX1280 %d] Triggers: txSendTrig=%d, idleToRxIdle=%d, "
    //               "txDoneTrig=%d, txTimeoutTrig=%d, "
    //               "idleRxTimeoutTrig=%d, rxActiveTrig=%d,
    //               txRxTimeoutTrig=%d\n", moduleId, (int)txSendTrig,
    //               (int)idleToRxIdle, (int)txDoneTrig, (int)txTimeoutTrig,
    //               (int)idleRxTimeoutTrig, (int)rxActiveTrig,
    //               (int)txRxTimeoutTrig);
    setDeadline(Core::NowNs());
  }
}

void Datalink_SX1280_V2::taskThread() {

  // Serial.printf("[SX1280 %d] Task thread Start. State: %d\n", moduleId,
  //               (int)state);

  auto threadStartTime = Core::NowNs();

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

  irqTrigTimestamp = 0; // reset for next DIO1 event
  lastRun = Core::NowNs();

  // Keep polling TX IRQ status while transmitting so TX_DONE is still detected
  // even if a DIO edge is missed; otherwise cadence can collapse to timeout
  // pacing.
  if (state == State::Transmitting && txStartTimestamp != 0 && !txDone &&
      !rxTxTimeout) {
    int64_t nextPoll = Core::NowNs() + 200 * Core::MICROSECONDS;
    int64_t timeoutAt = txStartTimestamp + txActiveTimeout;
    if (nextPoll > timeoutAt) {
      nextPoll = timeoutAt;
    }
    setRelease(nextPoll);
    setDeadline(nextPoll);
  } else {
    setRelease(Core::END_OF_TIME);
  }
}

void Datalink_SX1280_V2::fetchIrqFlags() {

  int64_t now = Core::NowNs();

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
  // In implicit header modes the SX1280 never fires IRQ_HEADER_VALID,
  // so only use headerValid as a trigger in Dynamic (explicit) mode.
  return rxDone || preambleDetected;
  // bool headerTrig = (packetMode == SX1280_PacketMode::Dynamic) &&
  // headerValid; return rxDone /* || preambleDetected*/ || crcError ||
  // headerTrig ||
  //        headerError;
}

bool Datalink_SX1280_V2::isTxReady() const {
  return sxTxPendingSize > 0 &&
         Core::NowNs() > (txScheduledTime - txPrepareLeadTime) &&
         state != State::Transmitting;
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
  // Serial.printf("[SX1280 %d] Frequency set to %lu Hz\n", moduleId,
  // freq_hz);
}

void Datalink_SX1280_V2::updateModParams() {
  if (modParamsChanged || freqChanged || packetParamsChanged) {
    // Serial.printf(
    //     "[SX1280 %d] updateModParams: modParamsChanged=%d, freqChanged=%d,
    //     " "packetParamsChanged=%d\n", moduleId, modParamsChanged,
    //     freqChanged, packetParamsChanged);
    lora.setMode(MODE_STDBY_XOSC);
    if (modParamsChanged) {
      lora.setModulationParams(spreadingFactor, bandwidth, codingRate);
      modParamsChanged = false;
    }
    if (freqChanged) {
      lora.setRfFrequency(freq_hz, 0);
      freqChanged = false;
    }
    if (packetParamsChanged) {
      if (packetMode == SX1280_PacketMode::Limited) {
        lora.setPacketParams(12, LORA_PACKET_FIXED_LENGTH,
                             fixedPacketLength + 1, LORA_CRC_ON,
                             LORA_IQ_NORMAL);
      } else if (packetMode == SX1280_PacketMode::Fixed) {
        lora.setPacketParams(12, LORA_PACKET_FIXED_LENGTH, fixedPacketLength,
                             LORA_CRC_ON, LORA_IQ_NORMAL);
      } else {
        lora.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON,
                             LORA_IQ_NORMAL);
      }
      packetParamsChanged = false;
    }
    state = State::Idle;
  }
}

void Datalink_SX1280_V2::prepareTx(const uint8_t *data, size_t size,
                                   int64_t txStart) {
  txScheduledTime = txStart == 0 ? Core::NowNs() : txStart;

  switch (packetMode) {
  case SX1280_PacketMode::Limited: {
    // OTA = 1-byte length prefix + fixedPacketLength bytes of payload area.
    size_t otaSize = fixedPacketLength + 1;
    sxTxPendingSize = otaSize;
    uint8_t lenByte = static_cast<uint8_t>(size);
    lora.startWriteSXBuffer(128);
    lora.writeBufferRaw(&lenByte, 1);
    lora.writeBufferRaw(data, size);
    // Pad remaining bytes with zeros.
    size_t padLen = fixedPacketLength - size;
    if (padLen > 0) {
      uint8_t zeros[padLen];
      memset(zeros, 0, padLen);
      lora.writeBufferRaw(zeros, padLen);
    }
    lora.endWriteSXBuffer();
    break;
  }
  case SX1280_PacketMode::Fixed: {
    // Write data padded to fixedPacketLength, no length prefix.
    sxTxPendingSize = fixedPacketLength;
    lora.startWriteSXBuffer(128);
    lora.writeBufferRaw(data, size);
    size_t padLen = fixedPacketLength - size;
    if (padLen > 0) {
      uint8_t zeros[padLen];
      memset(zeros, 0, padLen);
      lora.writeBufferRaw(zeros, padLen);
    }
    lora.endWriteSXBuffer();
    break;
  }
  case SX1280_PacketMode::Dynamic:
  default: {
    sxTxPendingSize = size;
    lora.startWriteSXBuffer(128);
    lora.writeBufferRaw(data, sxTxPendingSize);
    lora.endWriteSXBuffer();
    lora.setPayloadLength(static_cast<uint8_t>(sxTxPendingSize));
    break;
  }
  }

  // Compute effective power.
  int8_t power = txPower;
  if (power > maxTxPower)
    power = maxTxPower;
  power = power - (int8_t)paGain;

  if (power < -18) {
    power = -18;
  } else if (power > 12) {
    power = 12;
  }

  lora.setTxParams(power, RAMP_TIME);

  txPendingSize = 0;

  // Serial.printf("%.4f, Tx Prepared\n", Core::NOWSeconds());
}

void Datalink_SX1280_V2::startTx() {

  updateModParams();
  // TX can be entered from STDBY_RC, STDBY_XOSC, or FS modes.
  // With AutoFS enabled the radio is typically already in FS after
  // leaving continuous RX, so we only force STDBY if the state machine
  // thinks we're still in RX (shouldn't happen, but safety).
  if (state == State::IdleReceive || state == State::Receiving) {
    lora.setMode(MODE_STDBY_XOSC);
    state = State::Idle;
  }

  // Block until the scheduled tx time is reached.
  // Start tx should never be called more than the txPrepareLeadTime before
  // the scheduled time, so this should never be a long wait.
  auto waitStart = Core::NowNs();
  while (Core::NowNs() < txScheduledTime)
    ;
  txStartTimestamp = Core::NowNs();
  lora.setTx(txActiveTimeout / Core::MILLISECONDS);
  state = State::Transmitting;

  // double deltaMs = 0.0;
  // double busyWaiting = 0;
  // if (lastTxPrint != 0) {
  //   busyWaiting =
  //       (double)(txStartTimestamp - waitStart) / (double)Core::MILLISECONDS;
  //   deltaMs =
  //       (double)(txStartTimestamp - lastTxPrint) /
  //       (double)Core::MILLISECONDS;
  // }
  // Serial.print("SX1280 ");
  // Serial.print(moduleId);
  // Serial.print(" TX dt ms: ");
  // Serial.print(deltaMs, 3);
  // Serial.print(" busy wait ms: ");
  // Serial.println(busyWaiting, 3);
  // lastTxPrint = txStartTimestamp;

  txScheduledTime = 0;
}

void Datalink_SX1280_V2::startIdleRx() {
  // Use continuous RX — no hardware timeout.  The radio stays listening
  // until we explicitly change mode (for TX or channel hop).  This avoids
  // the RX→STDBY→RX cycling every 15ms that could desynchronise the
  // SX1280’s internal RX state machine (ELRS pattern).
  lora.setRxContinuous();
  state = State::IdleReceive;
  rxIdleStartTimestamp = Core::NowNs();
  rxStartedFlag = false;
  leaveRxFlag = false;
  // auto dTime =
  //     (double)(rxIdleStartTimestamp - lastRxPrint) / Core::MILLISECONDS;
  // Serial.printf(
  //     "[SX1280 %d] Entering IdleReceive state, time since last rx:
  //     %.3fs\n", moduleId, dTime);
  // lastRxPrint = rxIdleStartTimestamp;

  // Serial.printf("[SX1280 %d] Entering IdleReceive state.\n", moduleId);
}

void Datalink_SX1280_V2::startActiveRx() {
  // irqStatusRemain |=
  //     (crcError ? IRQ_CRC_ERROR : 0) | (headerError ? IRQ_HEADER_ERROR : 0)
  //     | (preambleDetected ? IRQ_PREAMBLE_DETECTED : 0) | (headerValid ?
  //     IRQ_HEADER_VALID : 0) | (rxDone ? IRQ_RX_DONE : 0);
  if (rxStartTimestamp == 0) {
    rxStartTimestamp = Core::NowNs();
  }
  acceptThisPacket = txRxEnabled;
  state = State::Receiving;
}

void Datalink_SX1280_V2::startIdle() {
  // Go to FS (frequency synthesis) instead of STDBY_RC so the PLL stays
  // locked and the next RX/TX entry is ~60µs instead of ~200µs.
  // With AutoFS enabled, we're usually already in FS after RX/TX, but
  // this handles cases where we need an explicit transition.
  lora.setMode(MODE_STDBY_XOSC);
  state = State::Idle;
}

void Datalink_SX1280_V2::retrieveRxData() {

  lastRxSuccessTime = Core::NowNs();

  receivedDataRSSI = receivedDataRSSI * 0.9 + lora.readPacketRSSI() * 0.1;
  receivedDataSNR = receivedDataSNR * 0.8 + lora.readPacketSNR() * 0.2;

  size_t otaLen;  // bytes read from radio buffer
  size_t userLen; // actual user payload length

  switch (packetMode) {
  case SX1280_PacketMode::Limited: {
    otaLen = fixedPacketLength + 1; // 1-byte length prefix + payload area
    uint8_t buffer[otaLen];
    lora.startReadSXBuffer(0);
    lora.readBuffer(buffer, otaLen);
    lora.endReadSXBuffer();
    userLen = buffer[0]; // first byte is length prefix
    if (userLen > fixedPacketLength) {
      userLen = fixedPacketLength; // sanity clamp
    }
    auto tOA = lora.getLoRaTimeOnAirMs(otaLen) * Core::MILLISECONDS;
    auto packetRecvStartTime = rxDoneTimestamp - tOA;
    DataPacket packet;
    packet.timestamp = packetRecvStartTime;
    packet.payload.setSize(userLen);
    memcpy(packet.payload.getPtr(), buffer + 1, userLen);
    receiveHandlers_.callHandlers(packet);
    return;
  }
  case SX1280_PacketMode::Fixed: {
    otaLen = fixedPacketLength;
    userLen = fixedPacketLength;
    uint8_t buffer[otaLen];
    lora.startReadSXBuffer(0);
    lora.readBuffer(buffer, otaLen);
    lora.endReadSXBuffer();
    auto tOA = lora.getLoRaTimeOnAirMs(otaLen) * Core::MILLISECONDS;
    auto packetRecvStartTime = rxDoneTimestamp - tOA;
    DataPacket packet;
    packet.timestamp = packetRecvStartTime;
    packet.payload.setSize(userLen);
    memcpy(packet.payload.getPtr(), buffer, userLen);
    receiveHandlers_.callHandlers(packet);
    return;
  }
  case SX1280_PacketMode::Dynamic:
  default: {
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
    receiveHandlers_.callHandlers(packet);
    return;
  }
  }
}

void Datalink_SX1280_V2::updateReceiveState() {

  if (rxDone) {
    // In implicit header modes the SX1280 never fires IRQ_HEADER_VALID,
    // so accept the packet as long as there's no CRC error.
    bool headerOk = (packetMode != SX1280_PacketMode::Dynamic) || headerValid;
    if (!crcError && headerOk && acceptThisPacket && !leaveRxFlag) {
      retrieveRxData();
    }
    rxDoneTimestamp = rxStartTimestamp = 0;
    startIdle();
    clearReceiveFlags();
  } else if (crcError || headerError || rxTxTimeout ||
             (Core::NowNs() - rxStartTimestamp > rxActiveTimeout) ||
             leaveRxFlag || !acceptThisPacket) {
    rxDoneTimestamp = rxStartTimestamp = 0;
    rxStartedFlag = false;
    startIdle();
    clearReceiveFlags();
  } else {
    return;
  }

  // After leaving RX, prepare+start any pending TX before entering IdleRx
  // to avoid a wasteful RX→STDBY round-trip.
  if (txPendingSize > 0 && sxTxPendingSize == 0) {
    prepareTx(txBuffer, txPendingSize, pendingTxTime);
  }
  if (isTxReady()) {
    startTx();
  } else if (rxStartedFlag) {
    updateModParams();
    startIdleRx();
  }
}

void Datalink_SX1280_V2::updateTransmitState() {

  bool timeout = Core::NowNs() - txStartTimestamp > txActiveTimeout;
  if (txDone || rxTxTimeout || timeout) {

    if (!txDone) {
      // TX failed or timed out — force back to a known state.
      // With AutoFS, the radio should be in FS after a failed TX,
      // but force STDBY_XOSC to be safe on a timeout.
      lora.setMode(MODE_STDBY_XOSC);
      state = State::Idle;
    } else {
      // Normal TX done — AutoFS puts us in FS already.
      state = State::Idle;
    }

    transmitFinishedHandler.callHandlers();
    sxTxPendingSize = 0;
    txStartTimestamp = 0;
    txDoneTimestamp = 0;
    rxTxTimeout = false;
    txDone = false;

    // Prepare any buffered TX before deciding to enter RX — avoids a
    // wasteful RX entry that would immediately be aborted by prepareTx.
    if (txPendingSize > 0) {
      prepareTx(txBuffer, txPendingSize, pendingTxTime);
    }
    if (isTxReady()) {
      startTx();
    } else if (rxStartedFlag) {
      updateModParams();
      startIdleRx();
    } else {
      startIdle();
    }
  }
}

void Datalink_SX1280_V2::updateIdleRxState() {

  if (txPendingSize > 0 && sxTxPendingSize == 0) {
    prepareTx(txBuffer, txPendingSize, pendingTxTime);
  }

  if (leaveRxFlag || !txRxEnabled) {
    leaveRxFlag = false;
    startIdle();
  } else if (isActivelyReceiving()) {
    // Highest priority: a packet is arriving — handle it now.
    startActiveRx();
    updateReceiveState();
  } else if (rxStartedFlag) {
    // Re-entering RX after a mod-param / freq change.
    updateModParams();
    startIdleRx();
  } else if (isTxReady()) {
    startTx();
  } else if (Core::NowNs() - rxIdleStartTimestamp > rxActiveTimeout) {
    // Software safety timeout — the radio is in continuous RX so this
    // should only fire if something is wrong (no packets AND no IRQs
    // for rxActiveTimeout).  Poll IRQ register to check if the radio
    // is still alive, and if we haven't received anything for >2s,
    // do a full hardware reconfigure.
    clearReceiveFlags();

    if (lastRxSuccessTime != 0 &&
        Core::NowNs() - lastRxSuccessTime > 2 * Core::SECONDS) {
      lastRxSuccessTime = Core::NowNs();
      fullReconfigure();
    }

    startIdleRx();
  }
}

void Datalink_SX1280_V2::updateIdleState() {

  // updateModParams();
  if (txPendingSize > 0 && sxTxPendingSize == 0) {
    prepareTx(txBuffer, txPendingSize, pendingTxTime);
  }

  if (isTxReady()) {
    startTx();
  } else if (rxStartedFlag) {
    updateModParams();
    startIdleRx();
  }
}

void Datalink_SX1280_V2::fullReconfigure() {
  // Hardware reset — toggles NRESET to force the SX1280 out of any
  // stuck internal state (e.g. desynchronised RX chain after many
  // consecutive timeouts in implicit-header mode).
  lora.resetDevice();

  lora.setupLoRa(freq_hz, 0, spreadingFactor, bandwidth, codingRate, false);
  lora.setHighSensitivity();
  lora.setBufferBaseAddress(128, 0);
  lora.setAutoFS(true);

  if (packetMode == SX1280_PacketMode::Limited) {
    lora.setPacketParams(12, LORA_PACKET_FIXED_LENGTH, fixedPacketLength + 1,
                         LORA_CRC_ON, LORA_IQ_NORMAL);
  } else if (packetMode == SX1280_PacketMode::Fixed) {
    lora.setPacketParams(12, LORA_PACKET_FIXED_LENGTH, fixedPacketLength,
                         LORA_CRC_ON, LORA_IQ_NORMAL);
  } else {
    lora.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON,
                         LORA_IQ_NORMAL);
  }
  lora.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE | IRQ_RX_DONE), 0, 0);

  modParamsChanged = false;
  freqChanged = false;
  packetParamsChanged = false;
  state = State::Idle;
  clearAllIrqFlags();
}

} // namespace VCTR::network::datalink