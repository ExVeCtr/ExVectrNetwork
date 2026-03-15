#ifndef EXVECTRNETWORK_DATAPACKET_HPP_
#define EXVECTRNETWORK_DATAPACKET_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/print.hpp"

namespace VCTR::network {

class PacketHeaderI;

/**
 * @brief General packet for all layers below the transport layer.
 * @details Each layer appends/pops its own header to the payload.
 */
class DataPacket {
  friend PacketHeaderI;

public:
  DataPacket() = default;

  DataPacket(const Core::ListArray<uint8_t> &other) : payload(other) {}

  /// The data carried by this packet.
  Core::ListArray<uint8_t> payload;

  /// The time when this packet was created for transmission or received.
  int64_t timestamp = 0;
};

/**
 * @brief defines the needed functions for a packet header.
 * @note The Header is always appended to the back of the payload.
 */
class PacketHeaderI {
public:
  virtual ~PacketHeaderI() {}

protected:
  /**
   * @brief Serializes the packet header into a list of bytes.
   */
  virtual void serialize(uint8_t *buffer) const = 0;
  /**
   * @brief Deserializes the packet header from a list of bytes.
   */
  virtual bool deserialize(const uint8_t *buffer) = 0;

public:
  /**
   * @brief Get the size of the header in bytes. Used to know how many bytes
   * to extract from the payload when popping the header.
   */
  virtual size_t getHeaderSize() const = 0;

  /// Helper functions for adding and removing headers from the payload.

  /// adds the header to the back of the packet.
  void addHeader(DataPacket &packet) {
    packet.payload.setSize(packet.payload.size() + getHeaderSize());
    serialize(packet.payload.getPtr() + packet.payload.size() -
              getHeaderSize());
  }

  /// Removes the header type from the back of the payload.
  void popHeader(DataPacket &packet) {
    const auto headerSize = getHeaderSize();
    if (packet.payload.size() < headerSize) {
      LOG_MSG("Not enough data to pop header! \n");
      return;
    }
    packet.payload.popDiscard(headerSize);
    return;
  }

  bool fromPacket(const DataPacket &packet) {
    if (packet.payload.size() < getHeaderSize()) {
      LOG_MSG("Not enough data to pop header! \n");
      return false;
    }
    deserialize(packet.payload.getPtr() + packet.payload.size() -
                getHeaderSize());
    return true;
  }
};

} // namespace VCTR::network

#endif