#include <cstring>

#include "ExVectrNetwork/datalink/sx1280/Sx1280_Direct.hpp"

namespace VCTR::network::datalink {

Sx1280_Direct::Sx1280_Direct(SX128XLT &sx1280Driver) : lora(sx1280Driver) {}

bool Sx1280_Direct::configureRadio() {
  if (!lora.checkDevice()) {
    return false;
  }

  lora.setupLoRa(freq_hz, 0, spreadingFactor, bandwidth, codingRate, false);
  lora.setHighSensitivity();
  lora.setBufferBaseAddress(kTxBufferAddress, kRxBufferAddress);
  lora.setPeriodBase(PERIODBASE_15_US);
  lora.setAutoFS(autoFSEnabled);
  // DIO1 mask: only fire the pin ISR on RX_DONE. The IRQ mask stays
  // IRQ_RADIO_ALL so preamble/header/CRC/timeout/TX_DONE are still visible
  // to fetchIrqFlags() via SPI polling (pull() is called unconditionally
  // every FHSS tick regardless of DIO1, so nothing is missed). TX_DONE is
  // deliberately left off DIO1 too: pull() never reads a timestamp for it,
  // and every flag other than RX_DONE ignores irqTrigTimestamp entirely, so
  // routing them to the pin only risked stealing the "first edge" latch in
  // notifyDio1Irq() from the one edge that's actually load-bearing -- the
  // exact instant of RX_DONE, which FHSS slot sync depends on.
  lora.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RX_DONE, 0, 0);
  applyPacketParams();
  lora.clearIrqStatus(IRQ_RADIO_ALL);
  clearIrqFlags();

  modParamsChanged = false;
  freqChanged = false;
  packetParamsChanged = false;
  state = State::Idle;
  return true;
}

void Sx1280_Direct::startRx(int64_t timeout) {
  const uint16_t clampedTimeout = clampRadioTimeout(timeout);

  // If we're already listening (nothing consumed the RX window: no rxDone/
  // txDone/timeout/error since we last armed it, and push() didn't just
  // reconfigure the radio -- both of those paths leave `state` at something
  // other than IdleReceive) with the same timeout, re-issuing setRx() would
  // just be a redundant checkBusy()+SPI round trip; the radio hardware is
  // already in the requested state. FHSS calls push()+startRx() on every
  // non-TX slot regardless of whether anything actually changed, so on a
  // quiet slot this was pure overhead -- and with two radios sharing one
  // physical SPI bus in diversity mode, that overhead is doubled every RX
  // slot, eating into the FHSS scheduling margin and increasing the
  // missed-slot rate. Skip it when it's provably a no-op.
  if (state == State::IdleReceive && clampedTimeout == lastRxTimeoutArmed) {
    return;
  }

  clearIrqFlags();
  lora.setRx(clampedTimeout);
  state = State::IdleReceive;
  lastRxTimeoutArmed = clampedTimeout;
}

int16_t Sx1280_Direct::getPacketRSSI() const { return receivedDataRSSI; }

int16_t Sx1280_Direct::getPacketSNR() const { return receivedDataSNR; }

network::DataPacket Sx1280_Direct::getRxPacket() const { return lastRxPacket; }

uint32_t Sx1280_Direct::getRxPacketCount() const { return rxPacketCount; }

bool Sx1280_Direct::setupTxPacket(const network::DataPacket &packet) {
  const size_t size = packet.payload.size();
  if (size == 0 || size > kMaxFrameLength) {
    return false;
  }

  if (packetMode == SX1280_PacketMode::Fixed && size != fixedPacketLength) {
    return false;
  }

  if (packetMode != SX1280_PacketMode::Dynamic && size > fixedPacketLength) {
    return false;
  }

  std::memcpy(txBuffer, packet.payload.getPtr(), size);
  txPendingSize = size;
  txPacketPending = true;
  txPacketLoaded = false;
  txLoadedSize = 0;
  return true;
}

void Sx1280_Direct::startTx() {
  if (!txPacketLoaded) {
    return;
  }

  clearIrqFlags();
  lora.setTx(kRadioTimeoutMax);
  txPacketLoaded = false;
  state = State::Transmitting;
}

uint32_t Sx1280_Direct::getTxPacketCount() const { return txPacketCount; }

uint8_t *Sx1280_Direct::getTxBufferPtr() { return txBuffer; }
uint8_t Sx1280_Direct::getTxBufferSize() const { return kMaxFrameLength; }

size_t Sx1280_Direct::getNumChannels() const { return kNumChannels; }

size_t Sx1280_Direct::getCurrentChannel() const { return currentChannel; }

void Sx1280_Direct::setChannel(size_t channel) {
  channel = channel % kNumChannels;
  currentChannel = static_cast<uint8_t>(channel);
  setFrequency(kMinFreq + channel * kChannelSpacing);
}

void Sx1280_Direct::setFrequency(uint32_t newFreqHz) {
  if (newFreqHz == freq_hz) {
    return;
  }

  freq_hz = newFreqHz;
  freqChanged = true;
}

void Sx1280_Direct::setSpreadingFactor(SX1280_SF sf) {
  if (spreadingFactor == sf) {
    return;
  }

  spreadingFactor = sf;
  modParamsChanged = true;
}

void Sx1280_Direct::setBandwidth(SX1280_BW bw) {
  if (bandwidth == bw) {
    return;
  }

  bandwidth = bw;
  modParamsChanged = true;
}

void Sx1280_Direct::setCodingRate(SX1280_CR cr) {
  if (codingRate == cr) {
    return;
  }

  codingRate = cr;
  modParamsChanged = true;
}

void Sx1280_Direct::setTxPower(int8_t power) { txPower = power; }

void Sx1280_Direct::setTxMaxPower(int8_t maxTxPower) {
  this->maxTxPower = maxTxPower;
}

void Sx1280_Direct::setPacketMode(SX1280_PacketMode mode) {
  if (packetMode == mode) {
    return;
  }

  packetMode = mode;
  packetParamsChanged = true;
}

void Sx1280_Direct::setFixedPacketLength(uint8_t length) {
  if (length == 0) {
    length = 1;
  } else if (length > kMaxFrameLength) {
    length = kMaxFrameLength;
  }

  if (fixedPacketLength == length) {
    return;
  }

  fixedPacketLength = length;
  packetParamsChanged = true;
}

void Sx1280_Direct::setPAdbm(uint8_t paDbm) { paGain = paDbm; }

void Sx1280_Direct::setAutoFS(bool enable) {
  autoFSEnabled = enable;
  if (isConfigured()) {
    lora.setAutoFS(enable);
  }
}

void Sx1280_Direct::setIdle() {
  lora.setMode(MODE_STDBY_XOSC);
  state = State::Idle;
}

void Sx1280_Direct::push(bool keepOscRunning) {
  // Only force a standby/reconfigure round trip (setMode + re-check mod/freq/
  // packet params) when something has actually changed. Deliberately does NOT
  // gate on `state != State::Idle`: startRx() always leaves state at
  // IdleReceive (never back to Idle), so that condition was true on every
  // single non-TX slot -- forcing a needless setMode(STDBY) -> reconfigure ->
  // (FHSS then calls startRx() right after) -> setRx() cycle every slot even
  // when nothing needed updating. startRx() itself (called unconditionally by
  // FHSS after push()) already re-arms the RX window every slot regardless,
  // so no correctness is lost by skipping this when truly idle-receiving.
  // This was cheap enough to not matter with one radio, but with two radios
  // sharing a single physical SPI bus (see Sx1280Diversity / SX1280.cpp's
  // shared `SPI` instance) it doubled real per-slot SPI bus time, tipping
  // FHSS slot scheduling over the edge under diversity.
  const bool needsRadioUpdate =
      modParamsChanged || freqChanged || packetParamsChanged || txPacketPending;

  if (!needsRadioUpdate) {
    return;
  }

  lora.setMode(keepOscRunning ? MODE_STDBY_XOSC : MODE_STDBY_RC);
  state = State::Idle;

  if (modParamsChanged) {
    lora.setModulationParams(spreadingFactor, bandwidth, codingRate);
    modParamsChanged = false;
  }

  if (freqChanged) {
    lora.setRfFrequency(freq_hz, 0);
    freqChanged = false;
  }

  if (packetParamsChanged) {
    applyPacketParams();
  }

  if (txPacketPending) {
    const bool packetMatchesMode =
        txPendingSize > 0 && ((packetMode == SX1280_PacketMode::Fixed &&
                               txPendingSize == fixedPacketLength) ||
                              (packetMode != SX1280_PacketMode::Fixed &&
                               txPendingSize <= getMaxPayloadSize()));

    if (packetMatchesMode) {
      prepareTxPacket(txBuffer, txPendingSize);
      txPacketLoaded = true;
    } else {
      txLoadedSize = 0;
      txPendingSize = 0;
      txPacketLoaded = false;
    }

    txPacketPending = false;
  }
}

void Sx1280_Direct::pull() {
  if (lora.checkAndClearBusyReset()) {
    // The low-level driver hard-reset the chip after a BUSY timeout
    // (SX128XLT::checkBusy()): every register is back at power-on defaults
    // while our staged flags still say "configured", so push() would never
    // reconfigure it. Rebuild the full configuration from the stored
    // settings (configureRadio() reapplies frequency/modulation/packet
    // params/DIO mask), otherwise this radio stays a zombie -- deaf, its
    // transmissions undecodable -- while a diversity layer keeps selecting
    // it for TX by its frozen last-known SNR, and each further command
    // to it risks another ~90 ms timeout+reset stall (missed FHSS slots).
    busyReinitPending = true;
  }

  if (busyReinitPending) {
    // Rate-limit the recovery: on a genuinely wedged chip every SPI command
    // (including configureRadio()'s own) burns the full ~90 ms
    // timeout+reset again, so retrying on every pull would stall the FHSS
    // scheduler continuously. Until a reconfigure succeeds the radio is
    // unusable anyway, so skip pull entirely between attempts.
    const int64_t now = Core::NowNs();
    if (now - lastBusyReinitAttemptNs < 1 * Core::SECONDS) {
      return;
    }
    lastBusyReinitAttemptNs = now;
    if (configureRadio()) {
      busyReinitPending = false;
      // Drop the reset flag a failed-then-successful configureRadio() may
      // have re-latched mid-way, so the next pull() doesn't restart the
      // recovery cycle on a now-healthy radio.
      lora.checkAndClearBusyReset();
    }
    return;
  }

  fetchIrqFlags();

  if (txDone) {
    ++txPacketCount;
    txLoadedSize = 0;
    clearIrqFlags();
    state = State::Idle;
    return;
  }

  if (rxDone) {
    const bool headerOk =
        packetMode != SX1280_PacketMode::Dynamic || headerValid;

    if (!crcError && !headerError && headerOk) {
      readCompletedPacketStatus();
      ++rxPacketCount;
    }

    clearIrqFlags();
    state = State::Idle;
    return;
  }

  if (rxTxTimeout || crcError || headerError) {
    if (state == State::Transmitting) {
      txLoadedSize = 0;
    }
    clearIrqFlags();
    state = State::Idle;
    return;
  }

  if (preambleDetected || headerValid) {
    state = State::Receiving;
  }
}

void Sx1280_Direct::notifyDio1Irq(int64_t timestamp, bool force) {
  if (force || irqTrigTimestamp == 0) {
    irqTrigTimestamp = timestamp;
  }
}

void Sx1280_Direct::fetchIrqFlags() {
  int64_t irqTimestamp = irqTrigTimestamp;
  if (irqTimestamp == 0 && dio1TimestampSource != nullptr) {
    // The DIO1 edge may have fired after the main loop last transferred ISR
    // captures but before this pull -- with RX_DONE-only on DIO1 the edge
    // lands near the end of the slot, so this race is common. Drain the
    // pending capture directly instead of falling back to NowNs() below,
    // which would be late by the loop latency and jitter the FHSS sync.
    irqTimestamp = dio1TimestampSource();
  }
  if (irqTimestamp == 0) {
    irqTimestamp = Core::NowNs();
  }

  const uint16_t irqStatus = lora.readIrqStatus();
  if (irqStatus == 0) {
    irqTrigTimestamp = 0;
    return;
  }

  uint16_t irqStatusSeen = 0;

  if (irqStatus & IRQ_PREAMBLE_DETECTED) {
    preambleDetected = true;
    irqStatusSeen |= IRQ_PREAMBLE_DETECTED;
  }

  if (irqStatus & IRQ_HEADER_VALID) {
    headerValid = true;
    irqStatusSeen |= IRQ_HEADER_VALID;
  }

  if (irqStatus & IRQ_HEADER_ERROR) {
    headerError = true;
    irqStatusSeen |= IRQ_HEADER_ERROR;
  }

  if (irqStatus & IRQ_CRC_ERROR) {
    crcError = true;
    irqStatusSeen |= IRQ_CRC_ERROR;
  }

  if (irqStatus & IRQ_RX_DONE) {
    rxDone = true;
    lastRxTimestamp = irqTimestamp;
    irqStatusSeen |= IRQ_RX_DONE;
  }

  if (irqStatus & IRQ_TX_DONE) {
    txDone = true;
    irqStatusSeen |= IRQ_TX_DONE;
  }

  if (irqStatus & IRQ_RX_TX_TIMEOUT) {
    rxTxTimeout = true;
    irqStatusSeen |= IRQ_RX_TX_TIMEOUT;
  }

  irqStatusRemain |= irqStatus & ~irqStatusSeen;
  lora.clearIrqStatus(irqStatus);
  irqTrigTimestamp = 0;
}

size_t Sx1280_Direct::getMaxPayloadSize() const {
  switch (packetMode) {
  case SX1280_PacketMode::Limited:
  case SX1280_PacketMode::Fixed:
    return fixedPacketLength;
  case SX1280_PacketMode::Dynamic:
  default:
    return kMaxFrameLength;
  }
}

int8_t Sx1280_Direct::getAppliedTxPower() const {
  int8_t power = txPower;
  if (power > maxTxPower) {
    power = maxTxPower;
  }

  power -= static_cast<int8_t>(paGain);

  if (power < -18) {
    power = -18;
  } else if (power > 12) {
    power = 12;
  }

  return power;
}

uint16_t Sx1280_Direct::clampRadioTimeout(int64_t timeout) const {
  if (timeout < 0) {
    return 0;
  }

  if (timeout > kRadioTimeoutMax) {
    return kRadioTimeoutMax;
  }

  return static_cast<uint16_t>(timeout);
}

void Sx1280_Direct::clearIrqFlags() {
  irqTrigTimestamp = 0;
  preambleDetected = false;
  headerValid = false;
  headerError = false;
  crcError = false;
  rxDone = false;
  txDone = false;
  rxTxTimeout = false;
  lastRxTimestamp = 0;
  irqStatusRemain = 0;
}

void Sx1280_Direct::applyPacketParams() {
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

  packetParamsChanged = false;
}

void Sx1280_Direct::prepareTxPacket(const uint8_t *data, size_t size) {
  switch (packetMode) {
  case SX1280_PacketMode::Limited: {
    uint8_t buffer[kMaxFrameLength + 1] = {0};
    buffer[0] = static_cast<uint8_t>(size);
    std::memcpy(buffer + 1, data, size);
    txLoadedSize = fixedPacketLength + 1;
    lora.directWriteSXBuffer(kTxBufferAddress, buffer,
                             static_cast<uint8_t>(txLoadedSize));
    break;
  }
  case SX1280_PacketMode::Fixed: {
    uint8_t buffer[kMaxFrameLength] = {0};
    std::memcpy(buffer, data, size);
    txLoadedSize = fixedPacketLength;
    lora.directWriteSXBuffer(kTxBufferAddress, buffer,
                             static_cast<uint8_t>(txLoadedSize));
    break;
  }
  case SX1280_PacketMode::Dynamic:
  default:
    txLoadedSize = size;
    lora.directWriteSXBuffer(kTxBufferAddress, data,
                             static_cast<uint8_t>(txLoadedSize));
    lora.setPayloadLength(static_cast<uint8_t>(txLoadedSize));
    break;
  }

  lora.setTxParams(getAppliedTxPower(), RAMP_TIME);
}

void Sx1280_Direct::readCompletedPacketStatus() {
  // Single combined status read instead of separate readPacketRSSI() +
  // readPacketSNR() calls -- see readPacketRSSISNR() for why (avoids 3
  // redundant RADIO_GET_PACKETSTATUS SPI+checkBusy() round trips down to 1).
  // This matters most in diversity mode, where two radios each pay this
  // cost on nearly every successfully-received slot.
  int8_t snr = 0;
  lora.readPacketRSSISNR(receivedDataRSSI, snr);
  receivedDataSNR = snr;

  // Only the cheap status is read here -- readRXPacketL() (Dynamic mode) is
  // itself just a RADIO_GET_RXBUFFERSTATUS status command, not a FIFO read.
  // The actual payload bytes are deliberately NOT fetched from the FIFO here;
  // that only happens in fetchRxPayload(), called once the caller (in
  // diversity, only after comparing RSSI/SNR across radios) actually needs
  // this packet's data. See fetchRxPayload()'s and Sx1280_DirectI's doc
  // comments for why.
  size_t otaLen;
  if (packetMode == SX1280_PacketMode::Limited) {
    pendingRxSize = fixedPacketLength;
    otaLen = fixedPacketLength + 1; // 1-byte length prefix on air
  } else if (packetMode == SX1280_PacketMode::Fixed) {
    pendingRxSize = fixedPacketLength;
    otaLen = fixedPacketLength;
  } else {
    uint8_t size = lora.readRXPacketL();
    if (size > kMaxFrameLength) {
      size = kMaxFrameLength;
    }
    pendingRxSize = size;
    otaLen = size;
  }

  // lastRxTimestamp is the DIO1 RX_DONE edge, i.e. the end of the packet
  // (guaranteed exact now that DIO1 is masked to RX_DONE only -- see
  // configureRadio()). FHSS::syncTimer() wants the packet START time (the
  // TX-side slot boundary), so subtract the deterministic time-on-air.
  const int64_t timeOnAir = static_cast<int64_t>(
      lora.getLoRaTimeOnAirMs(static_cast<uint8_t>(otaLen)) *
      static_cast<float>(Core::MILLISECONDS));
  lastRxPacket.timestamp = lastRxTimestamp - timeOnAir;

  rxPayloadPending = true;
}

void Sx1280_Direct::fetchRxPayload() {
  if (!rxPayloadPending) {
    return;
  }
  rxPayloadPending = false;

  if (packetMode == SX1280_PacketMode::Limited) {
    uint8_t buffer[kMaxFrameLength + 1] = {0};
    const size_t otaSize = fixedPacketLength + 1;
    lora.startReadSXBuffer(kRxBufferAddress);
    lora.readBuffer(buffer, static_cast<uint8_t>(otaSize));
    lora.endReadSXBuffer();

    size_t userSize = buffer[0];
    if (userSize > fixedPacketLength) {
      userSize = fixedPacketLength;
    }

    lastRxPacket.payload.setSize(userSize);
    std::memcpy(lastRxPacket.payload.getPtr(), buffer + 1, userSize);
    return;
  }

  if (packetMode == SX1280_PacketMode::Fixed) {
    uint8_t buffer[kMaxFrameLength] = {0};
    lora.startReadSXBuffer(kRxBufferAddress);
    lora.readBuffer(buffer, fixedPacketLength);
    lora.endReadSXBuffer();

    lastRxPacket.payload.setSize(fixedPacketLength);
    std::memcpy(lastRxPacket.payload.getPtr(), buffer, fixedPacketLength);
    return;
  }

  const uint8_t size = static_cast<uint8_t>(pendingRxSize);
  uint8_t buffer[kMaxFrameLength] = {0};
  lora.startReadSXBuffer(kRxBufferAddress);
  lora.readBuffer(buffer, size);
  lora.endReadSXBuffer();

  lastRxPacket.payload.setSize(size);
  std::memcpy(lastRxPacket.payload.getPtr(), buffer, size);
}

} // namespace VCTR::network::datalink