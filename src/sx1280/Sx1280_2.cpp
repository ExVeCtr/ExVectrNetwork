#include <cstdarg>
#include <cstdio>

#include "ExVectrCore/print.hpp"

#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/list.hpp"

#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/datalink/DatalinkI.hpp"

#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"
#include "ExVectrNetwork/datalink/sx1280/sx12xxAL/src/SX128XLT.h"

// #define SX1280_DEBUG

namespace VCTR::network::datalink {

Datalink_SX1280_V2::Datalink_SX1280_V2(SX128XLT &sx1280Driver)
    : Task_Periodic("Datalink_SX1280_V2", 100 * Core::MILLISECONDS),
      lora(sx1280Driver) {

  Core::getSystemScheduler().addTask(*this);
}

size_t Datalink_SX1280_V2::getMaxPacketSize() const { return 200; }

bool Datalink_SX1280_V2::isChannelBlocked() const {
  return radioState == RadioState::Receiving ||
         radioState == RadioState::Transmitting || transmitPacketSize > 0 ||
         !enableTxRx;
}

void Datalink_SX1280_V2::setFrequency(uint32_t freq_hz) {
  freqParamChanged = true;
  this->freq_hz = freq_hz;
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

void Datalink_SX1280_V2::setTxMaxPower(int8_t maxTxPowerDBm) {
  maxTxPowerDBm = maxTxPowerDBm;
}

void Datalink_SX1280_V2::setPAdbm(uint8_t paDbm) { paDbm = paDbm; }

void Datalink_SX1280_V2::setEnableTxRx(bool enable) {
  enableTxRx = enable;
  if (!enable) {
    lora.setMode(MODE_STDBY_RC);
    radioState = RadioState::Idle;
  }
}

void Datalink_SX1280_V2::addTransmitFinishedHandler(HandlerFunction handler) {
  transmitFinishedHandler.addHandler(handler);
}

void Datalink_SX1280_V2::taskInit() {

  if (!lora.checkDevice()) {
    LOG_MSG("Failed to initialize SX1280!\n");
    setInitialised(false);
    this->setRelease(Core::NOW() + 1 * Core::SECONDS);
    return;
  }

  lora.setupLoRa(2445000000, 0, LORA_SF6, LORA_BW_0800, LORA_CR_LI_4_8);
  lora.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0, 0);
  lora.setHighSensitivity();
  lora.receiveSXBuffer(0, 0, NO_WAIT);

  setPaused(true); // Pause the thread until dio interrupt

  radioState = RadioState::Idle;
}

void Datalink_SX1280_V2::taskCheck() {

  auto dio1State = lora.getDio1State();

  if (dio1State) {
    setRelease(Core::NOW());
  }
}

void Datalink_SX1280_V2::taskThread() {

  auto threadTime = Core::NOW();

  if (lora.getDio1State()) {

    LOG_MSG("DIO1 Pin is high!\n");

    uint16_t irqStatus = lora.readIrqStatus();

    events.clear();

    if ((irqStatus & IRQ_PREAMBLE_DETECTED)) {

      events.append(RadioEvent::CAD);

      LOG_MSG("Channel busy! Detected preamble\n");
    }

    if (irqStatus & (IRQ_HEADER_ERROR)) {

      LOG_MSG("Interrupt says header error!\n");

      events.append(RadioEvent::HeaderError);
    }

    if (irqStatus & (IRQ_CRC_ERROR)) {

      LOG_MSG("Interrupt says crc error!\n");

      events.append(RadioEvent::CRCError);
    }

    if (irqStatus & (IRQ_RX_TIMEOUT)) {

      LOG_MSG("Interrupt says rx timed out!\n");

      events.append(RadioEvent::RXTimeout);
    }

    if ((irqStatus & IRQ_RX_DONE) && (irqStatus & IRQ_HEADER_VALID) &&
        !(irqStatus & IRQ_CRC_ERROR)) {

      LOG_MSG("Received data!\n");

      events.append(RadioEvent::ReceiveDone);
    }

    if (irqStatus & (IRQ_TX_DONE)) {

      LOG_MSG("Data transmitted!\n");

      events.append(RadioEvent::TransmitDone);
    }

    if (irqStatus & (IRQ_TX_TIMEOUT)) {

      LOG_MSG("Interrupt says tx timed out!\n");

      events.append(RadioEvent::TXTimeout);
    }

    // lora.clearIrqStatus(IRQ_RADIO_ALL);

    if (irqStatus & (IRQ_CAD_ACTIVITY_DETECTED)) {

      LOG_MSG("Channel activity detected!\n");

      events.append(RadioEvent::CAD);
    }

    if (irqStatus & (IRQ_CAD_DONE)) {

      LOG_MSG("CAD done!\n");

      events.append(RadioEvent::CADDone);
    }
  } else {
    LOG_MSG("DIO1 Pin is low! \n");
  }

  if (events.size() > 0) {
    LOG_MSG("Processing events. Count: %d\n", events.size());

    if (eventsContain(RadioEvent::ReceiveDone)) {
      receiveAwaitingData();
    }

    if (eventsContain(RadioEvent::TransmitDone)) {
      transmitFinishedHandler.callHandlers();
    }

    if (eventsContain(RadioEvent::CADDone)) {
      if (radioState == RadioState::CAD) {
        if (eventsContain(RadioEvent::CAD)) {
          LOG_MSG("Channel was busy during CAD! Will not send data.\n");
          radioState = RadioState::Idle;
        } else if (transmitPacketSize > 0) {
          LOG_MSG("Channel was clear and we have data to send. Sending.\n");
          transmitAwaitingData();
        } else {
          LOG_MSG("Channel was clear. Checking again.\n");
          radioState = RadioState::CAD;
        }
      }
    }
  }
}

void Datalink_SX1280_V2::beginReceive() {

  radioState = RadioState::Receiving;

  updateModulationParams();

  lora.setMode(MODE_STDBY_RC);
  lora.setBufferBaseAddress(0, 0);
  lora.setDioIrqParams(IRQ_RADIO_ALL,
                       (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT + IRQ_HEADER_ERROR +
                        IRQ_PREAMBLE_DETECTED),
                       0, 0); // set for IRQ on RX done or timeout
  lora.setRx(0);
}

void Datalink_SX1280_V2::updateModulationParams() {
  if (modParamsChanged) {
#ifdef SX1280_DEBUG
    LOG_MSG("Changing mod params\n");
#endif
    lora.setModulationParams(spreadingFactor, bandwidth, codingRate);
    lora.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON,
                         LORA_IQ_NORMAL, 0, 0);
    modParamsChanged = false;
  }

  if (freqParamChanged) {
#ifdef SX1280_DEBUG
    LOG_MSG("Changing freq\n");
#endif
    lora.setRfFrequency(freq_hz, 0);
    freqParamChanged = false;
  }
}

void Datalink_SX1280_V2::transmitAwaitingData() {

#ifdef SX1280_DEBUG
  LOG_MSG("Transmitting data\n");
#endif

  if (transmitPacketSize == 0)
    return;

  updateModulationParams();
  auto power = txPower - paDbm;
  if (power > maxTxPowerDBm) {
    power = maxTxPowerDBm;
  }
  lora.transmitSXBuffer(0, transmitPacketSize, 0, power, NO_WAIT);
  transmitPacketSize = 0;
  radioState = RadioState::Transmitting;
  transmitStart = currentThreadStartTime;
}

void Datalink_SX1280_V2::receiveAwaitingData() {

  LOG_MSG("Retrieve data from radio\n");

  size_t packetL = lora.readRXPacketL();

  if (packetL > 0) { // make sure packet is okay

    receivedDataRSSI = lora.readPacketRSSI();
    receivedDataSNR = lora.readPacketSNR();

    // uint8_t buffer[packetL];
    // Core::ListArray<uint8_t> bufferArray;
    // bufferArray.setSize(packetL);
    // uint8_t crc = 0;

    DataPacket receivedData;
    receivedData.timestamp = receiveStart;
    receivedData.payload.setSize(packetL);

    lora.startReadSXBuffer(0);
    lora.readBuffer(receivedData.payload.getPtr(), packetL);
    lora.endReadSXBuffer();

    receiveHandlers_.callHandlers(receivedData);

  } else {
    LOG_MSG("Data was 0 bytes long! CRITICAL ERROR\n");
  }
}

void Datalink_SX1280_V2::beginCad() {
  updateModulationParams();

  lora.setMode(MODE_STDBY_RC);
  lora.setDioIrqParams(IRQ_RADIO_ALL, IRQ_CAD_DONE + IRQ_CAD_ACTIVITY_DETECTED,
                       0, 0);
  lora.startCAD(LORA_CAD_08_SYMBOL);
}

bool Datalink_SX1280_V2::transmitDataframe(const DataPacket &dataframe) {

  // LOG_MSG("Received %d bytes from topic to send. Pointer %d \n",
  // item.size(), this);

  if (!enableTxRx || transmitPacketSize > 0) {
    return false;
  }

  auto len = dataframe.payload.size();
  if (len > dataLinkMaxFrameLength) {
    LOG_MSG("Datalink: Max frame length exceeded. Failure.\n");
    return false;
  }

  LOG_MSG("Data received to send is %d bytes long. Adding to transmit buffer. "
          "Pointer %d \n",
          len, this);

  transmitPacketSize = len;
  lora.startWriteSXBuffer(0);
  lora.writeBuffer(dataframe.payload.getPtr(), len);
  lora.endWriteSXBuffer();

  return true;
}

size_t Datalink_SX1280_V2::getNumChannels() const { return numChannels; }

size_t Datalink_SX1280_V2::getCurrentChannel() const { return currentChannel; }

void Datalink_SX1280_V2::setChannel(size_t channel) {
  channel = channel % numChannels; // Wrap around the channel number if it
                                   // exceeds the number of channels
  currentChannel = channel;
  freq_hz = minFreq + channel * channelFrequencySpacing;
  setFrequency(freq_hz);
}

bool Datalink_SX1280_V2::eventsContain(RadioEvent event) {
  for (size_t i = 0; i < events.size(); ++i) {
    if (events[i] == event) {
      return true;
    }
  }
  return false;
}

} // namespace VCTR::network::datalink