#ifndef EXVECTRNETWORK_STRUCTS_NETWORKHEADER_HPP_
#define EXVECTRNETWORK_STRUCTS_NETWORKHEADER_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrNetwork/DataPacket.hpp"

namespace VCTR::network::network {

/// @brief   Network packet structure version. Added to the checksum.
static constexpr uint8_t networkVersion = 2;

enum class NetworkPacketType : uint8_t {
  DATA,     // Packet data is for the application layer.
  ACK,      // Packet data is an acknowledgement for a packet.
  NACK,     // Packet data is a negative acknowledgement for a packet.
  HEARTBEAT // Packet data is a heartbeat. Broadcasted after an interval
            // since last sent packet.
};

/**
 * @brief   Network packet structure.
 * @note    Raw data structure follows this format: [type, hops, dstAddress,
 * srcAddress, checksum, dataLength, data...] Usually only packet hops,
 * dstAddress, payloadLength and payload are used by the application layer. The
 * rest is handled by the network layer.
 */
class NetworkPacketHeader : public PacketHeaderI {
public:
  /// Packet type. What is this packet for?
  NetworkPacketType type = NetworkPacketType::DATA;
  /// Number of hops this packet can still take. Will be decremented by 1 each
  /// time it is forwarded. Set to 1 to prevent forwarding.
  uint8_t hops = 1;
  /// Destination address. Who receives this? Set to 0xFFFF to broadcast.
  uint16_t dstAddress;
  /// Source address. Who sent this?
  uint16_t srcAddress;
  /// Checksum. Used to verify the integrity of the packet. Calculated as the
  /// sum of all bytes in the packet only excluding the checksum byte plus
  /// network version number.
  uint8_t checksum;

protected:
  void serialize(uint8_t *buffer) const override;
  bool deserialize(const uint8_t *buffer) override;

public:
  size_t getHeaderSize() const override;
};

class NetworkPacket {
public:
  NetworkPacketHeader header;
  VCTR::network::DataPacket packet;
};

} // namespace VCTR::network::network

#endif