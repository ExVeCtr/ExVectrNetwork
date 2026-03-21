#ifndef SX128XLT_h
#define SX128XLT_h

#include "ExVectrHAL/digital_io.hpp"
#include "ExVectrHAL/pin_gpio.hpp"

#include "SX128XLT_Definitions.h"

namespace VCTR::network::datalink {

class SX128XLT {

public:
  SX128XLT(HAL::DigitalIO &spiBus, uint8_t device, HAL::PinGPIO &pinNSS,
           HAL::PinGPIO &pinNRESET, HAL::PinGPIO &pinRFBUSY,
           HAL::PinGPIO &pinDIO1, HAL::PinGPIO &pinRXEN, HAL::PinGPIO &pinTXEN);
  SX128XLT(HAL::DigitalIO &spiBus, uint8_t device, HAL::PinGPIO &pinNSS,
           HAL::PinGPIO &pinNRESET, HAL::PinGPIO &pinRFBUSY,
           HAL::PinGPIO &pinDIO1);

  bool begin();

  void rxEnable();
  void txEnable();

  void startCAD(uint8_t cadLength);
  bool getDio1State();

  void checkBusy();
  bool config();
  void readRegisters(uint16_t address, uint8_t *buffer, uint16_t size);
  uint8_t readRegister(uint16_t address);
  void writeRegisters(uint16_t address, uint8_t *buffer, uint16_t size);
  void writeRegister(uint16_t address, uint8_t value);
  void writeCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size);
  void readCommand(uint8_t Opcode, uint8_t *buffer, uint16_t size);
  void resetDevice();
  bool checkDevice();
  void setupLoRa(uint32_t frequency, int32_t offset, uint8_t modParam1,
                 uint8_t modParam2, uint8_t modParam3);
  void setMode(uint8_t modeconfig);
  void setRegulatorMode(uint8_t mode);
  void setPacketType(uint8_t PacketType);
  void setRfFrequency(uint32_t frequency, int32_t offset);
  void setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
  void setModulationParams(uint8_t modParam1, uint8_t modParam2,
                           uint8_t modParam3);
  void setPacketParams(uint8_t packetParam1, uint8_t packetParam2,
                       uint8_t packetParam3, uint8_t packetParam4,
                       uint8_t packetParam5, uint8_t packetParam6,
                       uint8_t packetParam7);
  void setPacketParams(uint8_t packetParam1, uint8_t packetParam2,
                       uint8_t packetParam3, uint8_t packetParam4,
                       uint8_t packetParam5);
  void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask,
                       uint16_t dio3Mask);
  void setHighSensitivity();
  void setLowPowerRX();
  void setLongPreamble(bool enable);
  void printModemSettings();
  void printDevice();
  uint32_t getFreqInt();
  uint8_t getLoRaSF();
  uint32_t returnBandwidth(uint8_t data);
  uint8_t getLoRaCodingRate();
  uint8_t getInvertIQ();
  uint16_t getPreamble();
  void printOperatingSettings();
  uint8_t getLNAgain();
  void printRegisters(uint16_t Start, uint16_t End);
  void printASCIIPacket(uint8_t *buff, uint8_t tsize);
  uint8_t transmit(uint8_t *txbuffer, uint8_t size, uint16_t timeout,
                   int8_t txpower, uint8_t wait);
  uint8_t transmitIRQ(uint8_t *txbuffer, uint8_t size, uint16_t timeout,
                      int8_t txpower, uint8_t wait);
  void setTxParams(int8_t TXpower, uint8_t RampTime);
  void setTx(uint16_t timeout);
  void clearIrqStatus(uint16_t irq);
  uint16_t readIrqStatus();
  void printIrqStatus();
  uint16_t CRCCCITT(uint8_t *buffer, uint32_t size, uint16_t start);
  uint8_t receive(uint8_t *rxbuffer, uint8_t size, uint16_t timeout,
                  uint8_t wait);
  uint8_t receiveIRQ(uint8_t *rxbuffer, uint8_t size, uint16_t timeout,
                     uint8_t wait);
  int16_t readPacketRSSI2();
  int16_t readPacketRSSI();
  int8_t readPacketSNR();
  uint8_t readRXPacketL();
  void setRx(uint16_t timeout);
  void setSyncWord1(uint32_t syncword);
  void setSyncWord2(uint32_t syncword);
  void setSyncWord3(uint32_t syncword);
  void setSyncWordErrors(uint8_t errors);
  void setSleep(uint8_t sleepconfig);
  uint16_t CRCCCITTSX(uint8_t startadd, uint8_t endadd, uint16_t startvalue);
  uint8_t getByteSXBuffer(uint8_t addr);
  int32_t getFrequencyErrorRegValue();
  int32_t getFrequencyErrorHz();
  void printHEXByte(uint8_t temp);
  void wake();
  uint8_t transmitAddressed(uint8_t *txbuffer, uint8_t size, char txpackettype,
                            char txdestination, char txsource, uint32_t timeout,
                            int8_t txpower, uint8_t wait);
  uint8_t receiveAddressed(uint8_t *rxbuffer, uint8_t size, uint16_t timeout,
                           uint8_t wait);
  uint8_t readRXPacketType();
  uint8_t readPacket(uint8_t *rxbuffer, uint8_t size);
  void setPeriodBase(uint8_t value);
  uint8_t getPeriodBase();

  //***************************************************************************
  // Start direct access SX buffer routines
  //***************************************************************************
  void startWriteSXBuffer(uint8_t ptr);
  uint8_t endWriteSXBuffer();
  void startReadSXBuffer(uint8_t ptr);
  uint8_t endReadSXBuffer();

  void writeUint8(uint8_t x);
  uint8_t readUint8();

  void writeInt8(int8_t x);
  int8_t readInt8();

  void writeInt16(int16_t x);
  int16_t readInt16();

  void writeUint16(uint16_t x);
  uint16_t readUint16();

  void writeInt32(int32_t x);
  int32_t readInt32();

  void writeUint32(uint32_t x);
  uint32_t readUint32();

  void writeFloat(float x);
  float readFloat();

  uint8_t readBuffer(uint8_t *rxbuffer, uint8_t size);

  uint8_t transmitSXBuffer(uint8_t startaddr, uint8_t length, uint16_t timeout,
                           int8_t txpower, uint8_t wait);
  uint8_t transmitSXBufferIRQ(uint8_t startaddr, uint8_t length,
                              uint16_t timeout, int8_t txpower, uint8_t wait);
  void writeBuffer(const uint8_t *txbuffer, uint8_t size);
  // void writeBuffer(uint8_t *txbuffer, uint8_t startaddr, uint8_t size);

  /**
   * @brief Write raw binary data into the SX buffer without appending a null
   * terminator. Unlike writeBuffer(), every byte in txbuffer is written
   * faithfully — the last byte is NOT overwritten with 0x00.
   * Use this for binary payloads when paired with readBuffer(ptr, size).
   */
  void writeBufferRaw(const uint8_t *txbuffer, uint8_t size);

  uint8_t receiveSXBuffer(uint8_t startaddr, uint16_t timeout, uint8_t wait);
  uint8_t receiveSXBufferIRQ(uint8_t startaddr, uint16_t timeout, uint8_t wait);
  uint8_t readBuffer(uint8_t *rxbuffer);
  // uint8_t readBuffer(uint8_t *rxbuffer, uint8_t startaddr, uint8_t len);
  void printSXBufferHEX(uint8_t start, uint8_t end);
  void writeBufferChar(char *txbuffer, uint8_t size);
  uint8_t readBufferChar(char *rxbuffer);

  void setupFLRC(uint32_t frequency, int32_t offset, uint8_t modParam1,
                 uint8_t modParam2, uint8_t modParam3, uint32_t syncword);
  void setFLRCPayloadLengthReg(uint8_t length);
  void setLoRaPayloadLengthReg(uint8_t length);
  void setPayloadLength(uint8_t length);

  //***************************************************************************
  // LoRa Time-on-Air calculation helpers
  //***************************************************************************

  /**
   * @brief Calculate the total number of LoRa symbols for a given payload size,
   *        using the currently saved modulation and packet parameters.
   *
   * This implements the formulas from SX1280 datasheet section 7.4.4.1
   * (legacy coding rate, i.e. not Long Interleaving).
   *
   * @param payloadBytes  Number of application payload bytes.
   * @return Total symbol count (preamble + header + payload + CRC).
   */
  float getLoRaSymbolCount(uint8_t payloadBytes);

  /**
   * @brief Calculate the LoRa time-on-air in milliseconds for a given payload
   *        size, using the currently saved modulation and packet parameters.
   *
   * @param payloadBytes  Number of application payload bytes.
   * @return Time-on-air in milliseconds (as float for sub-ms precision).
   */
  float getLoRaTimeOnAirMs(uint8_t payloadBytes);

  /**
   * @brief Calculate the total number of LoRa symbols for fully explicit
   *        parameters (does not rely on saved state).
   *
   * Uses the legacy (non-Long-Interleaving) formula from datasheet 7.4.4.1.
   *
   * @param sf             Spreading factor (5 – 12).
   * @param cr             Coding rate numerator offset (1=4/5, 2=4/6, 3=4/7,
   * 4=4/8).
   * @param preambleSymbols Number of preamble symbols.
   * @param headerType     true = variable/explicit header, false =
   * fixed/implicit.
   * @param crcOn          true = 16-bit CRC appended, false = no CRC.
   * @param payloadBytes   Number of application payload bytes.
   * @return Total symbol count as float.
   */
  static float calcLoRaSymbolCount(uint8_t sf, uint8_t cr,
                                   uint16_t preambleSymbols, bool headerType,
                                   bool crcOn, uint8_t payloadBytes);

  /**
   * @brief Calculate LoRa time-on-air in milliseconds for fully explicit
   *        parameters.
   *
   * @param sf             Spreading factor (5 – 12).
   * @param bandwidthHz    Bandwidth in Hz (e.g. 812500).
   * @param cr             Coding rate numerator offset (1=4/5 … 4=4/8).
   * @param preambleSymbols Number of preamble symbols.
   * @param headerType     true = variable/explicit, false = fixed/implicit.
   * @param crcOn          true = CRC on.
   * @param payloadBytes   Number of application payload bytes.
   * @return Time-on-air in milliseconds.
   */
  static float calcLoRaTimeOnAirMs(uint8_t sf, uint32_t bandwidthHz, uint8_t cr,
                                   uint16_t preambleSymbols, bool headerType,
                                   bool crcOn, uint8_t payloadBytes);

private:
  HAL::PinGPIO &_NSS, &_NRESET, &_RFBUSY, &_DIO1;
  HAL::PinGPIO &_RXEN, &_TXEN;
  uint8_t _RXPacketL;     // length of packet received
  uint8_t _RXPacketType;  // type number of received packet
  uint8_t _RXDestination; // destination address of received packet
  uint8_t _RXSource;      // source address of received packet
  // int8_t  _PacketRSSI;          //RSSI of received packet - removed November
  // 2021, not used int8_t  _PacketSNR;           //signal to noise ratio of
  // received packet - removed November 2021, not used
  int8_t _TXPacketL; // transmitted packet length
  uint8_t _RXcount;  // used to keep track of the bytes read from SX1280 buffer
                     // during readFloat() etc
  uint8_t _TXcount;  // used to keep track of the bytes written to SX1280 buffer
                     // during writeFloat() etc
  uint8_t _OperatingMode;    // current operating mode
  bool _rxtxpinmode = false; // set to true if RX and TX pin mode is used.

  uint8_t _Device;    // saved device type
  uint8_t _TXDonePin; // the pin that will indicate TX done
  uint8_t _RXDonePin; // the pin that will indicate RX done
  uint8_t _PERIODBASE = PERIODBASE_01_MS;

  uint8_t savedRegulatorMode;
  uint8_t savedPacketType;
  uint32_t savedFrequency;
  int32_t savedOffset;
  uint8_t savedModParam1, savedModParam2,
      savedModParam3; // sequence is spreading factor, bandwidth, coding rate
  uint8_t savedPacketParam1, savedPacketParam2, savedPacketParam3,
      savedPacketParam4, savedPacketParam5, savedPacketParam6,
      savedPacketParam7;
  uint16_t savedIrqMask, savedDio1Mask, savedDio2Mask, savedDio3Mask;
  int8_t savedTXPower;
  uint16_t savedCalibration;
  uint32_t savedFrequencyReg;

  uint8_t _ReliableErrors; // Reliable status byte
  uint8_t _ReliableFlags;  // Reliable flags byte
  uint8_t _ReliableConfig; // Reliable config byte

  HAL::DigitalIO &_spiBus;
};

} // namespace VCTR::network::datalink
#endif
