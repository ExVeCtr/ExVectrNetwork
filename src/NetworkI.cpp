#include "ExVectrCore/print.hpp"

#include "ExVectrNetwork/network/NetworkI.hpp"

namespace VCTR::network::network /* NetworkPacketHeader */ {

void NetworkPacketHeader::serialize(uint8_t *buffer) const {
  const auto headerSize = getHeaderSize();

  buffer[0] = static_cast<uint8_t>(type);
  buffer[1] = hops;
  buffer[2] = static_cast<uint8_t>(dstAddress >> 8);
  buffer[3] = static_cast<uint8_t>(dstAddress & 0xFF);
  buffer[4] = static_cast<uint8_t>(srcAddress >> 8);
  buffer[5] = static_cast<uint8_t>(srcAddress & 0xFF);

  // Calculate header checksum
  uint8_t checksum = 0; // Core::computeCrc(data, 0);
  for (size_t i = 0; i < headerSize; i++) {
    if (i == 6) // Skip checksum byte
      continue;
    checksum += buffer[i];
  }
  buffer[6] = checksum + networkVersion;
}

bool NetworkPacketHeader::deserialize(const uint8_t *buffer) {
  const auto headerSize = getHeaderSize();

  type = static_cast<NetworkPacketType>(buffer[0]);
  hops = buffer[1];
  dstAddress = (buffer[2] << 8) | buffer[3];
  srcAddress = (buffer[4] << 8) | buffer[5];
  // Byte 6 is checksum
  uint8_t checksum = 0; // Core::computeCrc(data, 0);
  for (size_t i = 0; i < headerSize; i++) {
    if (i == 6) // Skip checksum byte
      continue;
    checksum += buffer[i];
  }
  if (checksum != (uint8_t)(buffer[6] + networkVersion)) {
    LOG_MSG("Header checksum failed! Expected: %d, Is: %d \n", buffer[6],
            checksum);
    return false;
  }

  return true;
}

size_t NetworkPacketHeader::getHeaderSize() const { return 7; }

} // namespace VCTR::network::network

namespace VCTR::network::network /* NetworkI */ {

NetworkI::NetworkI(uint16_t nodeAddress) : nodeAddress_(nodeAddress) {}

void NetworkI::setNodeAddress(uint16_t nodeAddress) {
  nodeAddress_ = nodeAddress;
}

uint16_t NetworkI::getNodeAddress() const { return nodeAddress_; }

void NetworkI::addPacketReceiveHandler(
    Core::HandlerGroup<const NetworkPacketHeader &,
                       const Core::ListArray<uint8_t> &>::HandlerFunction
        handler) {
  packetReceiveHandlers_.addHandler(handler);
}

void NetworkI::clearPacketReceiveHandlers() {
  packetReceiveHandlers_.clearHandlers();
}

} // namespace VCTR::network::network