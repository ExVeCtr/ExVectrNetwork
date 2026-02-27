#include "ExVectrCore/print.hpp"

#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/list.hpp"

#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/datalink/DatalinkI.hpp"

#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"
#include "ExVectrNetwork/datalink/sx1280/sx12xxAL/src/SX128XLT.h"

#include <cstdarg>
#include <cstdio>

// #define SX1280_DEBUG

namespace VCTR::network::datalink {

Datalink_SX1280::Datalink_SX1280(SX128XLT &sx1280Driver, const char *name)
    : Task_Periodic("Datalink_SX1280", 100 * Core::MILLISECONDS),
      lora(sx1280Driver), name(name) {

  Core::getSystemScheduler().addTask(*this);
}

void Datalink_SX1280::logMsg(const char *format, ...) const {
  char message[256];
  va_list args;
  va_start(args, format);
  vsnprintf(message, sizeof(message), format, args);
  va_end(args);
  LOG_MSG("[%s] %s", name, message);
}

void Datalink_SX1280::debugLog(const char *format, ...) const {
#ifdef SX1280_DEBUG
  char message[256];
  va_list args;
  va_start(args, format);
  vsnprintf(message, sizeof(message), format, args);
  va_end(args);
  LOG_MSG("[%s] %s", name, message);
#endif
}

size_t Datalink_SX1280::getBufferFreeSpace() const {
  return transmitBuffer_.sizeMax() - transmitBuffer_.size();
}

size_t Datalink_SX1280::getMaxPacketSize() const {
  return getBufferFreeSpace();
}

bool Datalink_SX1280::isChannelBlocked() const {
  return radioState_ == RadioState::ChannelBusy ||
         radioState_ == RadioState::Receiving ||
         radioState_ == RadioState::ActivityDetection ||
         radioState_ == RadioState::Transmitting || transmitBuffer_.size() > 0;
}

void Datalink_SX1280::setFrequency(uint32_t freq_hz) {
  freqParamChanged = true;
  this->freq_hz = freq_hz;
}

void Datalink_SX1280::setSpreadingFactor(SX1280_SF sf) {
  modParamsChanged = true;
  spreadingFactor = sf;
}

void Datalink_SX1280::setBandwidth(SX1280_BW bw) {
  modParamsChanged = true;
  bandwidth = bw;
}

void Datalink_SX1280::setCodingRate(SX1280_CR cr) {
  modParamsChanged = true;
  codingRate = cr;
}

void Datalink_SX1280::setTxPower(float power) { txPower_ = power; }

void Datalink_SX1280::enableTxRx(bool enable) {
  enableTxRx_ = enable;
  if (!enable && radioState_ == RadioState::Receiving) {
    lora.setMode(MODE_STDBY_RC);
    radioState_ = RadioState::Idle;
  }
}

void Datalink_SX1280::cadBeforeSend(bool enable) { cadBeforeSend_ = enable; }

void Datalink_SX1280::clearTransmitBuffer() { transmitBuffer_.clear(); }

void Datalink_SX1280::taskInit() {

  if (!lora.checkDevice()) {
    logMsg("Failed to initialize SX1280!\n");
    setInitialised(false);
    this->setRelease(Core::NOW() + 1 * Core::SECONDS);
    return;
  }

  lora.setupLoRa(2445000000, 0, LORA_SF5, LORA_BW_1600, LORA_CR_4_6);
  lora.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0, 0);
  lora.setHighSensitivity();
  lora.receiveSXBuffer(0, 0, NO_WAIT);

  setPaused(true); // Pause the thread until dio interrupt

  radioState_ = RadioState::Idle;
}

void Datalink_SX1280::taskCheck() {

  if (lora.getDio1State() ||
      (transmitBuffer_.size() > 0 &&
       (radioState_ == RadioState::Idle ||
        radioState_ ==
            RadioState::WaitingForReceive))) { // If the dio pin is high or we
                                               // have data to
    // send, then we should run the task
    setPaused(false);
    setRelease(Core::NOW());
  }
}

void Datalink_SX1280::taskThread() {

  auto threadTime = Core::NOW();

  if (lora.getDio1State()) {

    debugLog("DIO1 Pin is high!\n");

    uint16_t irqStatus = lora.readIrqStatus();

    if ((irqStatus &
         IRQ_PREAMBLE_DETECTED)) { // We have detected a preamble. This means
                                   // we are receiving data.

      radioState_ = RadioState::ChannelBusy;
      channelBusyStart_ = threadTime;

      debugLog("Channel busy! Detected preamble\n");
      lora.clearIrqStatus(IRQ_PREAMBLE_DETECTED);
    }

    if ((irqStatus & IRQ_RX_DONE) && (irqStatus & IRQ_HEADER_VALID) &&
        !(irqStatus & IRQ_CRC_ERROR)) {

      debugLog("Received data! Decoding...\n");

      receiveAwaitingData();
      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      receiveEnd_ = threadTime;

      lora.clearIrqStatus(IRQ_RX_DONE + IRQ_HEADER_VALID +
                          IRQ_PREAMBLE_DETECTED); // Clear the irq status. We
                                                  // are done receiving data.
    }

    if (irqStatus & (IRQ_HEADER_ERROR)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      receiveEnd_ = threadTime;

      logMsg("Interrupt says header error!\n");

      lora.clearIrqStatus(IRQ_HEADER_ERROR); // Clear the irq status. We are
                                             // done receiving data.
    }

    if (irqStatus & (IRQ_CRC_ERROR)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      receiveEnd_ = threadTime;

      logMsg("Interrupt says crc error!\n");

      lora.clearIrqStatus(
          IRQ_CRC_ERROR); // Clear the irq status. We are done receiving data.
    }

    if (irqStatus & (IRQ_RX_TIMEOUT)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      receiveEnd_ = threadTime;

      logMsg("Interrupt says rx timed out!\n");

      lora.clearIrqStatus(IRQ_RX_TIMEOUT); // Clear the irq status. We are
                                           // done receiving data.
    }

    if (irqStatus & (IRQ_TX_DONE)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      transmitEnd_ = threadTime;

      debugLog("Interrupt says tx done!\n");

      // LOG_MSG("Tx took %.2f ms\n",
      //         double(transmitEnd_ - transmitStart_) / Core::MILLISECONDS);

      lora.clearIrqStatus(
          IRQ_TX_DONE); // Clear the irq status. We are done receiving data.
    }

    if (irqStatus & (IRQ_TX_TIMEOUT)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      transmitEnd_ = threadTime;

      debugLog("Interrupt says tx timeout!\n");

      lora.clearIrqStatus(IRQ_TX_TIMEOUT); // Clear the irq status. We are
                                           // done receiving data.
    }

    // lora.clearIrqStatus(IRQ_RADIO_ALL);

    if (irqStatus & (IRQ_CAD_ACTIVITY_DETECTED)) {
      // radioState_ = RadioState::ActivityDetection;
      channelBusyStart_ = threadTime;

      beginReceive(1000);

      radioState_ = RadioState::Receiving;
      // lora.receiveSXBuffer(0, 0, NO_WAIT); //Set to receive mode. Use
      // 0xFFFF to go into continuous receive mode.

      // lora.startCAD(LORA_CAD_08_SYMBOL);

      debugLog("Channel busy! Channel activity detected! Checking again\n");

      lora.clearIrqStatus(
          IRQ_CAD_ACTIVITY_DETECTED); // Clear the irq status. We are done
                                      // receiving data.
    }

    if (irqStatus & (IRQ_CAD_DONE)) {

      lora.clearIrqStatus(
          IRQ_CAD_DONE); // Clear the irq status. We are done receiving data.

      if (radioState_ == RadioState::ChannelBusy) {
        channelBusyEnd_ = threadTime;
        debugLog("Channel is free again\n");
      }
      channelBusyEnd_ = threadTime;
      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;

      debugLog("Channel is free!\n");

      transmitAwaitingData(threadTime);
    }
  } else {
    debugLog("DIO1 Pin is low! \n");
  }

  /*if (radioState_ == RadioState::ChannelBusy) { // The channel is busy. We
  should immediatly start receiving as we want this data.

      radioState_ = RadioState::Receiving;
      beginReceive();
      receiveStart_ = threadTime;

      logMsg("Channel is busy. Beginning receive to collect the data\n");

  }*/

  if (transmitBuffer_.size() > 0 &&
      (radioState_ == RadioState::Idle ||
       radioState_ == RadioState::WaitingForReceive) &&
      threadTime - transmitEnd_ > 1 * Core::MILLISECONDS) {

    if (cadBeforeSend_) {
      radioState_ = RadioState::ActivityDetection;
      lora.startCAD(LORA_CAD_08_SYMBOL);
    } else {
      transmitAwaitingData(threadTime);
    }
  }

  if (radioState_ == RadioState::Idle && enableTxRx_) {
    beginReceive();
    receiveStart_ = threadTime;

    debugLog("Radio is idle. Beginning receive to collect possible "
             "transmissions\n");
  }

  if (radioState_ == RadioState::Receiving &&
      threadTime - receiveStart_ > 500 * Core::MILLISECONDS && enableTxRx_) {
    beginReceive();
    receiveStart_ = threadTime;

    debugLog("Radio RX timeout issue. Putting back into receive\n");
  }

  if (radioState_ == RadioState::Transmitting &&
      threadTime - transmitStart_ >
          500 * Core::MILLISECONDS) { // Timout case for transmitting data. Go
                                      // into idle as something went wrong

    radioState_ = RadioState::Idle;

    debugLog("Radio TX timeout issue. Putting back into idle\n");
  }

  if ((radioState_ == RadioState::ChannelBusy ||
       radioState_ == RadioState::ActivityDetection) &&
      threadTime - channelBusyStart_ >
          500 * Core::MILLISECONDS) { // Timout case for channel busy. Go into
                                      // idle as something went wrong

    radioState_ = RadioState::Idle;

    debugLog("Radio Channel busy timeout issue. Putting back into idle\n");
  }
}

void Datalink_SX1280::beginReceive(int timeout) {

  radioState_ = RadioState::WaitingForReceive; // Set the radio to waiting for
                                               // receive mode.

  lora.setMode(MODE_STDBY_RC);     // Set the radio to standby mode. This will
                                   // allow us to receive data.
  lora.setBufferBaseAddress(0, 0); // order is TX RX
  lora.setDioIrqParams(IRQ_RADIO_ALL,
                       (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT + IRQ_HEADER_ERROR +
                        IRQ_PREAMBLE_DETECTED),
                       0, 0); // set for IRQ on RX done or timeout
  updateModulationParams();
  lora.setRx(timeout); // set to receive mode
}

void Datalink_SX1280::updateModulationParams() {
  if (modParamsChanged) {
    lora.setModulationParams(spreadingFactor, bandwidth, codingRate);
    lora.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON,
                         LORA_IQ_NORMAL, 0, 0);
    modParamsChanged = false;
  }

  if (freqParamChanged) {
    lora.setRfFrequency(freq_hz, 0);
    freqParamChanged = false;
  }
}

bool Datalink_SX1280::transmitAwaitingData(int64_t threadTime) {

  if (transmitBuffer_.size() == 0)
    return false; // Nothing to send. Return.
  // if (radioState_ != RadioState::Idle) return false; //Radio is not ready
  // to send data. Return.

  const uint8_t packetLimit = 250;
  uint8_t buffer[packetLimit];
  // uint8_t bufferSize = 0;
  // uint8_t crc = 0;

  debugLog("There is data to send. Moving segments into buffer...\n");

  // We keep placing each packet frame into the buffer untill the next one
  // would be too much.
  // while (transmitBuffer_.size() > 0 &&
  //        bufferSize + transmitBuffer_[0] + 2 <= packetLimit &&
  //        transmitBuffer_[0] !=
  //            0) { // The next addition of bytes would be the frame size plus
  //                 // also the frame size number. This number is needed by
  //                 // receiver to decode. An addistion byte for the crc.

  //   auto frameLen = transmitBuffer_[0];
  //   debugLog("Segment is %d bytes long\n", frameLen);

  //   // transmitBuffer_.removeFront(); //Dont remove this. This is needed by
  //   // the
  //   for (int i = 0; i < frameLen + 1; i++) {
  //     buffer[i + bufferSize] = transmitBuffer_[i];
  //     crc += buffer[i + bufferSize];
  //   }
  //   transmitBuffer_.removeFront(frameLen + 1);
  //   bufferSize += frameLen + 1;
  // }
  // buffer[bufferSize] = crc;
  // bufferSize += 1;

  /*transmitBuffer_.takeFront(bufferSize); //Get the first byte. This is the
  length of the frame.

  if (bufferSize == 0) {
      logMsg("Datalink: No data to send. Failure.\n");
      transmitBuffer_.clear(); //Clear the buffer. Something is wrong.
      return false; //No data to send. Failure.
  }

  if (bufferSize > packetLimit) {
      logMsg("Datalink: Buffer overflow. Failure.\n");
      transmitBuffer_.clear(); //Clear the buffer. Something is wrong.
      return false; //Buffer overflow case. Failure.
  }

  for (size_t i = 0; i < bufferSize; i++) { //We have to move all elements
  ahead the one to be removed, one place down. buffer[i] = transmitBuffer_[i];
      //crc += buffer[i];
  }
  //bufferSize = transmitBuffer_.size();
  transmitBuffer_.removeFront(bufferSize); //Remove the data bytes from the
  buffer.*/

  for (size_t i = 0; i < transmitBuffer_.size() && i < packetLimit; i++) {
    buffer[i] = transmitBuffer_[i];
  }

  // debugLog("Finished making buffer. Sending data! Buffer length: %d\n",
  //          bufferSize);

  if (/*bufferSize > 0*/ true) {
    lora.transmit(buffer, transmitBuffer_.size(), 0, txPower_, NO_WAIT);
    transmitBuffer_.clear(); // Clear the buffer. We have sent the data.
    radioState_ = RadioState::Transmitting;
    transmitStart_ = threadTime;
    return true;
  } else {
    logMsg("Datalink: No data to send. Failure. Why does the buffer contain "
           "data, "
           "but the data is marked with 0 length?... Buffer will be cleared\n");
    transmitBuffer_.clear();
  }

  return false; // No data to send. Failure.
}

void Datalink_SX1280::receiveAwaitingData() {

  debugLog("Interrupt says data received\n");

  size_t packetL = lora.readRXPacketL();

  if (packetL > 0) { // make sure packet is okay

    receivedDataRSSI_ = lora.readPacketRSSI();
    receivedDataSNR_ = lora.readPacketSNR();

    // uint8_t buffer[packetL];
    // Core::ListArray<uint8_t> bufferArray;
    // bufferArray.setSize(packetL);
    // uint8_t crc = 0;

    DataPacket receivedData;
    receivedData.payload.setSize(packetL);

    lora.startReadSXBuffer(0);
    lora.readBuffer(receivedData.payload.getPtr(), packetL);
    lora.endReadSXBuffer();

    receiveHandlers_.callHandlers(receivedData);
    return;

    // receiveTopic_.publish(bufferArray);

    // uint8_t crc = 0;
    // for (int i = 0; i < packetL - 1; i++)
    //   crc += bufferArray[i];
    // uint8_t crcRcv = bufferArray[packetL - 1];

    // if (crc == crcRcv) { // Only decode if crc is correct.

    //   // Core::ListBuffer<uint8_t, dataLinkMaxFrameLength> receivedData;
    //   DataPacket receivedData;
    //   for (size_t i = 0; i < packetL - 1;) {

    //     auto segLen = bufferArray[i];
    //     i++;                               // Read the size
    //     receivedData.payload.clear();      // Make sure its empty
    //     for (int j = 0; j < segLen; j++) { // Place data into list.
    //       receivedData.payload.append(bufferArray[j + i]);
    //     }
    //     debugLog("Data segment Received is %d bytes long.\n", segLen);
    //     receiveHandlers_.callHandlers(receivedData);

    //     i += segLen;
    //   }
    // } else {
    //   logMsg("Received data is corrupt and cant be decoded reliably! Data len
    //   "
    //          "%d, crcRcv %d, crc calc %d\n",
    //          packetL, crcRcv, crc);
    // }
  } else {
    logMsg("Data was 0 bytes long! CRITICAL ERROR\n");
  }
}

bool Datalink_SX1280::transmitDataframe(const DataPacket &dataframe) {

  // LOG_MSG("Received %d bytes from topic to send. Pointer %d \n",
  // item.size(), this);

  if (!enableTxRx_) {
    return false;
  }

  auto len = dataframe.payload.size();
  if (len > dataLinkMaxFrameLength) {
    logMsg("Datalink: Max frame length exceeded. Failure.\n");
    return false; // Max frame length exceeded. Failure.
  }
  if (len > transmitBuffer_.sizeMax() - transmitBuffer_.size() - 1) {
    logMsg("Datalink: Buffer overflow. Failure.\n");
    return false; // Buffer overflow case. Failure.
  }

  debugLog("Data received to send is %d bytes long. Adding to transmit buffer. "
           "Pointer %d \n",
           len, this);

  // transmitBuffer_.placeBack(len);
  for (size_t i = 0; i < len; i++)
    transmitBuffer_.placeBack(dataframe.payload[i]);
  return true;
}

size_t Datalink_SX1280::getNumChannels() const { return numChannels; }

size_t Datalink_SX1280::getCurrentChannel() const { return currentChannel_; }

void Datalink_SX1280::setCurrentChannel(size_t channel) {
  channel = channel % numChannels; // Wrap around the channel number if it
                                   // exceeds the number of channels
  currentChannel_ = channel;
  freq_hz = minFreq + channel * channelFrequencySpacing;
}

} // namespace VCTR::network::datalink