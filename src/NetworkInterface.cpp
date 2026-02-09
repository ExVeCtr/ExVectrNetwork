#include "ExVectrNetwork/interfaces/NetworkInterface.hpp"

namespace VCTR::Net {

Network_Interface::Network_Interface(uint16_t nodeAddress)
    : nodeAddress_(nodeAddress) {}

void Network_Interface::setNodeAddress(uint16_t nodeAddress) {
  nodeAddress_ = nodeAddress;
}

uint16_t Network_Interface::getNodeAddress() const { return nodeAddress_; }

void Network_Interface::addPacketReceiveHandler(
    Core::HandlerGroup<const NetworkPacket &>::HandlerFunction handler) {
  packetReceiveHandlers_.addHandler(handler);
}

void Network_Interface::clearPacketReceiveHandlers() {
  packetReceiveHandlers_.clearHandlers();
}

} // namespace VCTR::Net