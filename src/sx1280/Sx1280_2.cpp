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

uint8_t Datalink_SX1280_V2::moduleCount_ = 0;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

Datalink_SX1280_V2::Datalink_SX1280_V2(SX128XLT &sx1280Driver)
    : Task_Periodic("Datalink_SX1280_V2", 100 * Core::MILLISECONDS),
      lora_(sx1280Driver) {
  moduleId_ = moduleCount_++;
  Core::getSystemScheduler().addTask(*this);
}

// ---------------------------------------------------------------------------
// DatalinkI / RadioI overrides
// ---------------------------------------------------------------------------

size_t Datalink_SX1280_V2::getMaxPacketSize() const { return kMaxFrameLength; }

bool Datalink_SX1280_V2::isChannelBlocked() const {
  return state_ == State::Transmitting || txPendingSize_ > 0 || !enableTxRx_ ||
         preambleActive_;
}

bool Datalink_SX1280_V2::transmitDataframe(const DataPacket &dataframe) {
  if (!enableTxRx_ || txPendingSize_ > 0) {
    return false;
  }

  auto len = dataframe.payload.size();
  if (len > kMaxFrameLength) {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] Frame too large (%d > %d)\n", moduleId_,
                  (int)len, (int)kMaxFrameLength);
#endif
    return false;
  }

#ifdef SX1280_DEBUG
  Serial.printf("[SX1280 %d] Queued %d bytes for TX\n", moduleId_, (int)len);
#endif

  // Write the payload into the SX1280's internal buffer while we are still
  // in whatever mode we are in. The buffer is a separate memory space and
  // can be written at any time.
  //
  // NOTE: We use writeBufferRaw (not writeBuffer) to avoid the library's
  // null-terminator behaviour, which overwrites the last payload byte with
  // 0x00.  writeBufferRaw writes every byte faithfully.
  lora_.startWriteSXBuffer(0);
  lora_.writeBufferRaw(dataframe.payload.getPtr(), len);
  lora_.endWriteSXBuffer();

  txPendingSize_ = len;
  return true;
}

size_t Datalink_SX1280_V2::getNumChannels() const { return kNumChannels; }
size_t Datalink_SX1280_V2::getCurrentChannel() const { return currentChannel_; }

void Datalink_SX1280_V2::setChannel(size_t channel) {
  channel = channel % kNumChannels;
  currentChannel_ = channel;
  freqLast_hz_ = freq_hz_;
  freq_hz_ = kMinFreq + channel * kChannelSpacing;

  // Apply the frequency change immediately if the radio is idle-receiving
  // (no preamble in progress, not transmitting, DIO1 clear).  This avoids
  // a 1-5 ms scheduling delay that would otherwise eat into the FHSS hop
  // advance margin on every single hop.
  if (enableTxRx_ && state_ == State::Receiving && !preambleActive_ &&
      !lora_.getDio1State() && txPendingSize_ == 0) {
    freqChanged_ = true;
    enterRx(); // STDBY → apply freq → back to RX, ~200 µs via SPI
  } else {
    freqChanged_ = true; // defer to next task cycle
  }
}

// ---------------------------------------------------------------------------
// Configuration setters
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::setFrequency(uint32_t freq_hz) {
  freqChanged_ = true;
  freqLast_hz_ = freq_hz_;
  freq_hz_ = freq_hz;
}

void Datalink_SX1280_V2::setSpreadingFactor(SX1280_SF sf) {
  modParamsChanged_ = true;
  spreadingFactorLast_ = spreadingFactor_;
  spreadingFactor_ = sf;
}

void Datalink_SX1280_V2::setBandwidth(SX1280_BW bw) {
  modParamsChanged_ = true;
  bandwidthLast_ = bandwidth_;
  bandwidth_ = bw;
}

void Datalink_SX1280_V2::setCodingRate(SX1280_CR cr) {
  modParamsChanged_ = true;
  codingRateLast_ = codingRate_;
  codingRate_ = cr;
}

void Datalink_SX1280_V2::setTxPower(int8_t power) { txPower_ = power; }

void Datalink_SX1280_V2::setTxMaxPower(int8_t maxTxPower) {
  maxTxPower_ = maxTxPower;
}

void Datalink_SX1280_V2::setPAdbm(uint8_t paDbm) { paDbm_ = paDbm; }

void Datalink_SX1280_V2::setEnableTxRx(bool enable) {
  if (enable && enableTxRx_) {
    // Already enabled — do nothing.  Calling enterRx() here would abort
    // an ongoing reception or transmission.
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] setEnableTxRx(true) — already enabled, "
                  "ignoring\n",
                  moduleId_);
#endif
    return;
  }

  enableTxRx_ = enable;
  if (!enable) {
    lora_.setMode(MODE_STDBY_RC);
    state_ = State::Idle;
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] setEnableTxRx(false) — radio disabled\n",
                  moduleId_);
#endif
  } else {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] setEnableTxRx(true) — enabling\n", moduleId_);
#endif
    enterRx();
  }
}

void Datalink_SX1280_V2::addTransmitFinishedHandler(
    std::function<void()> handler) {
  transmitFinishedHandler_.addHandler(handler);
}

// ---------------------------------------------------------------------------
// Task: Init
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::taskInit() {
  if (!lora_.checkDevice()) {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] Device check FAILED\n", moduleId_);
#endif
    setInitialised(false);
    this->setRelease(Core::NOW() + 1 * Core::SECONDS);
    return;
  }

#ifdef SX1280_DEBUG
  Serial.printf("[SX1280 %d] Device found, configuring\n", moduleId_);
#endif

  lora_.setupLoRa(freq_hz_, 0, spreadingFactor_, bandwidth_, codingRate_);
  lora_.setLongPreamble(true);
  lora_.setHighSensitivity();

  // Mark as applied so enterRx() doesn't redo them immediately.
  modParamsChanged_ = false;
  freqChanged_ = false;

  state_ = State::Idle;
  if (enableTxRx_) {
    enterRx();
  }
}

// ---------------------------------------------------------------------------
// Task: Check  (called frequently between threads to allow early wake)
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::taskCheck() {
  // Wake the task immediately when there is work to do:
  //  - DIO1 high means a radio IRQ is pending (RX_DONE, TX_DONE, etc.)
  //  - txPendingSize_ > 0 means data is queued for transmission
  //  - param/freq changes need to be applied
  auto irqState = lora_.getDio1State();
  if (irqState && irqTrigTimestamp_ == 0) {
    irqTrigTimestamp_ = Core::NOW();
  }
  if (irqState || txPendingSize_ > 0 || modParamsChanged_ || freqChanged_) {
    setRelease(Core::NOW());
  }
}

// ---------------------------------------------------------------------------
// Task: Thread  (the main periodic body)
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::taskThread() {
  if (!enableTxRx_) {
    return;
  }

  threadStartTime_ = Core::NOW();

  // ------------------------------------------------------------------
  // 1. Read and *immediately clear* any pending hardware IRQ bits.
  //
  //    We clear ONLY the bits we just read.  If a new event (e.g.
  //    IRQ_RX_DONE) fires between our read and our clear, its bit is
  //    NOT in our mask and therefore survives the clear — no lost
  //    events.
  //
  //    Clearing is essential: without it DIO1 stays permanently high,
  //    taskCheck() continuously calls setRelease(NOW()), and we spin-
  //    read the SPI bus hundreds of times per millisecond.  That
  //    aggressive polling during an active reception can push the
  //    SX1280 into an anomalous state where it stops generating
  //    IRQ_RX_DONE after a few packets.
  //
  //    Because intermediate events (PREAMBLE_DETECTED → HEADER_VALID
  //    → RX_DONE) arrive on separate DIO1 edges now, we OR every read
  //    into irqAccum_ so the state machine sees the full picture.
  //    The accumulator is reset on every state transition (enterRx /
  //    startTransmit).
  // ------------------------------------------------------------------
  uint16_t irq = 0;
  if (lora_.getDio1State()) {
    irq = lora_.readIrqStatus();
    lora_.clearIrqStatus(irq);
  }
  irqAccum_ |= irq;

  // Track when we last saw any IRQ activity (for the watchdog).
  if (irq != 0) {
    lastIrqActivityTime_ = threadStartTime_;

#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] IRQ 0x%04X  accum 0x%04X\n", moduleId_, irq,
                  irqAccum_);
#endif
  }

  // ------------------------------------------------------------------
  // 2. Handle events based on current state.
  // ------------------------------------------------------------------
  switch (state_) {

  // --- RECEIVING -----------------------------------------------------
  case State::Receiving: {
    // Record preamble-detect timestamp (for FHSS sync).
    // Use freshly-read `irq` so we capture the time of the *first*
    // detection, not a later poll where the bit is only in the accum.
    if (irq & IRQ_PREAMBLE_DETECTED) {
#ifdef SX1280_DEBUG
      Serial.printf("[SX1280 %d] Preamble detected\n", moduleId_);
#endif
      receiveStartTimestamp_ = irqTrigTimestamp_;
      preambleActive_ = true;
    }

    // Packet fully received.
    if ((irqAccum_ & IRQ_RX_DONE) && (irqAccum_ & IRQ_HEADER_VALID) &&
        !(irqAccum_ & IRQ_CRC_ERROR)) {
#ifdef SX1280_DEBUG
      Serial.printf("[SX1280 %d] RX_DONE — valid packet\n", moduleId_);
#endif
      recvFinishTimestamp_ = irqTrigTimestamp_;
      handleReceivedPacket();
      enterRx(); // back to continuous RX
      break;
    }

    // Errors — discard and restart RX.
    if (irqAccum_ & (IRQ_HEADER_ERROR | IRQ_CRC_ERROR | IRQ_RX_TIMEOUT)) {
      preambleActive_ = false;
#ifdef SX1280_DEBUG
      if (irqAccum_ & IRQ_HEADER_ERROR)
        Serial.printf("[SX1280 %d] Header error\n", moduleId_);
      if (irqAccum_ & IRQ_CRC_ERROR)
        Serial.printf("[SX1280 %d] CRC error\n", moduleId_);
      if (irqAccum_ & IRQ_RX_TIMEOUT)
        Serial.printf("[SX1280 %d] RX timeout\n", moduleId_);
#endif
      enterRx();
      break;
    }

    // Software watchdog — if we started accumulating IRQ bits (e.g.
    // preamble detected) but never got RX_DONE or an error, the radio
    // may be stuck.  Force a restart after 500 ms of inactivity.
    if (irqAccum_ != 0 &&
        (threadStartTime_ - lastIrqActivityTime_) > 500 * Core::MILLISECONDS) {
#ifdef SX1280_DEBUG
      Serial.printf("[SX1280 %d] Watchdog — stale accum 0x%04X, restarting "
                    "RX\n",
                    moduleId_, irqAccum_);
#endif
      enterRx();
      break;
    }

    // If params changed while idle-receiving (no packet in flight),
    // restart RX so the new params take effect.
    if ((modParamsChanged_ || freqChanged_) &&
        !(irqAccum_ & IRQ_PREAMBLE_DETECTED)) {
      enterRx();
      break;
    }

    // If we have data to send AND no packet is currently being received,
    // switch to transmit.
    if (txPendingSize_ > 0 && !(irqAccum_ & IRQ_PREAMBLE_DETECTED) &&
        !(irqAccum_ & IRQ_HEADER_VALID)) {
      startTransmit();
    }
    break;
  }

  // --- TRANSMITTING --------------------------------------------------
  case State::Transmitting: {
    if (irqAccum_ & IRQ_TX_DONE) {
#ifdef SX1280_DEBUG
      Serial.printf("[SX1280 %d] TX_DONE\n", moduleId_);
#endif
      transmitFinishedHandler_.callHandlers();
      enterRx();
      break;
    }

    // TX timeout safety net.
    if ((irqAccum_ & IRQ_TX_TIMEOUT) ||
        (threadStartTime_ - transmitStartTime_ > 500 * Core::MILLISECONDS)) {
#ifdef SX1280_DEBUG
      Serial.printf("[SX1280 %d] TX timeout\n", moduleId_);
#endif
      lora_.setMode(MODE_STDBY_RC);
      enterRx();
    }
    break;
  }

  // --- IDLE (should not stay here long) ------------------------------
  case State::Idle:
  default:
    enterRx();
    break;
  }

  irqTrigTimestamp_ = 0; // reset for next DIO1 event
}

// ---------------------------------------------------------------------------
// Private: Calculate LoRa packet airtime (nanoseconds)
// ---------------------------------------------------------------------------

int64_t Datalink_SX1280_V2::calcPacketAirtime(size_t payloadBytes) const {
  // Derive numeric SF from the register value (0x50 = SF5 … 0xC0 = SF12).
  const int sf = static_cast<uint8_t>(spreadingFactor_) >> 4;

  // Derive bandwidth in Hz from the register value.
  double bw_hz;
  switch (static_cast<uint8_t>(bandwidth_)) {
  case LORA_BW_0200:
    bw_hz = 203125.0;
    break;
  case LORA_BW_0400:
    bw_hz = 406250.0;
    break;
  case LORA_BW_0800:
    bw_hz = 812500.0;
    break;
  case LORA_BW_1600:
    bw_hz = 1625000.0;
    break;
  default:
    bw_hz = 812500.0;
    break;
  }

  // Derive coding-rate denominator from the register value.
  // CR 4/5 = 0x01, 4/6 = 0x02, 4/7 = 0x03, 4/8 = 0x04
  // LI 4/5 = 0x05, LI 4/6 = 0x06, LI 4/8 = 0x07
  int cr_denom;
  switch (static_cast<uint8_t>(codingRate_)) {
  case LORA_CR_4_5:
  case LORA_CR_LI_4_5:
    cr_denom = 5;
    break;
  case LORA_CR_4_6:
  case LORA_CR_LI_4_6:
    cr_denom = 6;
    break;
  case LORA_CR_4_7:
    cr_denom = 7;
    break;
  case LORA_CR_4_8:
  case LORA_CR_LI_4_8:
    cr_denom = 8;
    break;
  default:
    cr_denom = 8;
    break;
  }

  // Symbol time in seconds: T_sym = 2^SF / BW
  const double t_sym = (double)(1 << sf) / bw_hz;

  // Preamble time.  We use 12 preamble symbols (from setPacketParams) plus
  // the SX1280 adds 4.25 symbols overhead.
  const double n_preamble = 12.0;
  const double t_preamble = (n_preamble + 4.25) * t_sym;

  // Payload symbol count (Semtech SX1280 datasheet formula).
  // explicitHeader = true (variable-length packets), CRC on.
  const int pl = static_cast<int>(payloadBytes);
  const int crcBits = 16;    // CRC on
  const int headerBits = 20; // explicit header overhead

  // Total bits to encode: payload + CRC + header overhead
  // For SF5/SF6 the SX1280 always uses 8-symbol minimum.
  int numBitPayload = 8 * pl + crcBits + headerBits;
  double numSymPayload;

  if (sf <= 6) {
    // No low data rate optimisation; ceil division
    numSymPayload =
        8.0 +
        (double)((numBitPayload > 4 * sf)
                     ? ((numBitPayload - 4 * sf + 4 * (cr_denom)-5) /
                        (4 * (cr_denom - 4) > 0 ? 4 * (cr_denom - 4) : 1)) *
                               (cr_denom) +
                           8
                     : 8);
  } else {
    // Standard formula for SF7–SF12
    double val = (double)(8 * pl - 4 * sf + 8 + crcBits + headerBits);
    if (val < 0)
      val = 0;
    double symbolsPayload = 8.0 + ceil(val / (4.0 * sf)) * cr_denom;
    numSymPayload = symbolsPayload;
  }

  const double t_payload = numSymPayload * t_sym;
  const double t_total = t_preamble + t_payload;

  // Convert seconds → nanoseconds.
  return static_cast<int64_t>(t_total * 1e9);
}

// ---------------------------------------------------------------------------
// Private: Apply parameter changes
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::applyParamChanges() {
  if (modParamsChanged_) {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] Applying mod params\n", moduleId_);
#endif
    lora_.setModulationParams(spreadingFactor_, bandwidth_, codingRate_);
    lora_.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON,
                          LORA_IQ_NORMAL, 0, 0);
    modParamsChanged_ = false;
  }
  if (freqChanged_) {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] Applying freq %lu\n", moduleId_,
                  (unsigned long)freq_hz_);
#endif
    lora_.setRfFrequency(freq_hz_, 0);
    freqChanged_ = false;
  }
}

// ---------------------------------------------------------------------------
// Private: Enter continuous RX
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::enterRx() {
#ifdef SX1280_DEBUG
  Serial.printf("[SX1280 %d] enterRx() from state %d, accum 0x%04X\n",
                moduleId_, (int)state_, irqAccum_);
#endif

  // Reset the software accumulator — we are starting a fresh RX cycle.
  irqAccum_ = 0;
  preambleActive_ = false;

  // Go to STDBY so we can safely reconfigure.
  lora_.setMode(MODE_STDBY_RC);

  // Apply any pending modulation / frequency changes.
  applyParamChanges();

  // Configure buffer pointers.
  lora_.setBufferBaseAddress(0, 0);

  // Route the IRQs we care about to DIO1.
  lora_.setDioIrqParams(IRQ_RADIO_ALL,
                        (IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT |
                         IRQ_PREAMBLE_DETECTED | IRQ_HEADER_VALID |
                         IRQ_HEADER_ERROR | IRQ_CRC_ERROR),
                        0, 0);

  // Enter continuous RX (timeout = 0xFFFF means no timeout on the SX1280).
  lora_.setRx(0xFFFF);

  state_ = State::Receiving;

#ifdef SX1280_DEBUG
  Serial.printf("[SX1280 %d] Entered RX\n", moduleId_);
#endif
}

// ---------------------------------------------------------------------------
// Private: Read completed packet
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::handleReceivedPacket() {
  size_t packetLen = lora_.readRXPacketL();
  if (packetLen == 0) {
#ifdef SX1280_DEBUG
    Serial.printf("[SX1280 %d] RX packet length 0 — ignoring\n", moduleId_);
#endif
    return;
  }

  receivedDataRSSI_ = lora_.readPacketRSSI();
  receivedDataSNR_ = lora_.readPacketSNR();

  // Back-calculate the packet transmission start time from the current time
  // (RX_DONE moment) and the computed on-air time for this packet.
  int64_t airTime = lora_.getLoRaTimeOnAirMs(packetLen) * Core::MILLISECONDS;
  Serial.printf("[SX1280 %d] Packet airtime %.3f ms, length: %d bytes\n",
                moduleId_, (double)airTime / Core::MILLISECONDS,
                (int)packetLen);
  int64_t rxDoneTime =
      recvFinishTimestamp_ == 0 ? threadStartTime_ : recvFinishTimestamp_;
  receiveStartTimestamp_ = rxDoneTime - airTime;

  DataPacket pkt;
  pkt.timestamp = receiveStartTimestamp_;
  pkt.payload.setSize(packetLen);

  lora_.startReadSXBuffer(0);
  lora_.readBuffer(pkt.payload.getPtr(), packetLen);
  lora_.endReadSXBuffer();

#ifdef SX1280_DEBUG
  Serial.printf("[SX1280 %d] Dispatching %d-byte packet (RSSI %d, SNR %d)\n",
                moduleId_, (int)packetLen, (int)receivedDataRSSI_,
                (int)receivedDataSNR_);
#endif

  receiveHandlers_.callHandlers(pkt);
}

// ---------------------------------------------------------------------------
// Private: Start TX
// ---------------------------------------------------------------------------

void Datalink_SX1280_V2::startTransmit() {
  if (txPendingSize_ == 0)
    return;

  // Reset the software accumulator — we are starting a fresh TX cycle.
  irqAccum_ = 0;

#ifdef SX1280_DEBUG
  Serial.printf("[SX1280 %d] Starting TX (%d bytes)\n", moduleId_,
                (int)txPendingSize_);
#endif

  // Go to STDBY so we can reconfigure for TX.
  lora_.setMode(MODE_STDBY_RC);

  // Apply any pending param changes before transmitting.
  applyParamChanges();

  // Compute effective power.
  int8_t power = txPower_ - (int8_t)paDbm_;
  if (power > maxTxPower_)
    power = maxTxPower_;

  // Configure buffer and payload length.
  lora_.setBufferBaseAddress(0, 0);
  lora_.setPayloadLength(static_cast<uint8_t>(txPendingSize_));
  lora_.setTxParams(power, RAMP_TIME);

  // Route TX-related IRQs to DIO1.
  lora_.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT), 0, 0);

  // Start TX (no timeout — we have a software timeout).
  lora_.setTx(0);

  transmitStartTime_ = Core::NOW();
  state_ = State::Transmitting;
  txPendingSize_ = 0;
}

} // namespace VCTR::network::datalink