#ifndef EXVECTRNETWORK_STRUCTS_TRANSPORTHEADER_HPP_
#define EXVECTRNETWORK_STRUCTS_TRANSPORTHEADER_HPP_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrNetwork/DataPacket.hpp"

namespace VCTR::network::transport {

/// @brief   Network packet structure version. Added to the checksum.
static constexpr uint8_t networkVersion = 2;

/**
 * @brief   Network packet structure.
 * @note    Raw data structure follows this format: [type, hops, dstAddress,
 * srcAddress, checksum, dataLength, data...] Usually only packet hops,
 * dstAddress, payloadLength and payload are used by the application layer. The
 * rest is handled by the network layer.
 */
class TransportHeader : public PacketHeaderI {
public:
  uint16_t srcAddress;
  uint16_t srcPort;
  uint16_t dstAddress;
  uint16_t dstPort;

protected:
  void serialize(uint8_t *buffer) const override;
  bool deserialize(const uint8_t *buffer) override;

public:
  size_t getHeaderSize() const override;
};

} // namespace VCTR::network::transport

#endif