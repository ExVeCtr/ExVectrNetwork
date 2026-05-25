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
  lora.setAutoFS(false);
  lora.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0, 0);
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
  clearIrqFlags();
  lora.setRx(clampRadioTimeout(timeout));
  state = State::IdleReceive;
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

void Sx1280_Direct::push(bool keepOscRunning) {
  clearIrqFlags();
  lora.clearIrqStatus(IRQ_RADIO_ALL);
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
      readCompletedPacket();
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

void Sx1280_Direct::readCompletedPacket() {
  receivedDataRSSI = lora.readPacketRSSI();
  receivedDataSNR = lora.readPacketSNR();
  lastRxPacket.timestamp = lastRxTimestamp;

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

  uint8_t size = lora.readRXPacketL();
  if (size > kMaxFrameLength) {
    size = kMaxFrameLength;
  }

  uint8_t buffer[kMaxFrameLength] = {0};
  lora.startReadSXBuffer(kRxBufferAddress);
  lora.readBuffer(buffer, size);
  lora.endReadSXBuffer();

  lastRxPacket.payload.setSize(size);
  std::memcpy(lastRxPacket.payload.getPtr(), buffer, size);
}

} // namespace VCTR::network::datalink