#ifndef EXVECTRNETWORK_INTERFACES_NETWORKINTERFACE_HPP_
#define EXVECTRNETWORK_INTERFACES_NETWORKINTERFACE_HPP_

#include "ExVectrCore/handler.hpp"

#include "ExVectrNetwork/interfaces/DatalinkInterface.hpp"
#include "ExVectrNetwork/structs/NetworkPacket.hpp"

namespace VCTR {

namespace Net {

/**
 * @brief Network layer class. This class takes care of routing packets to their
 * destination.
 */
class Network_Interface {
protected:
  /// The address of this node.
  uint16_t nodeAddress_;

  Core::HandlerGroup<const NetworkPacket &> packetReceiveHandlers_;

public:
  /**
   * @brief Construct a new Network Node object.
   *
   * @param nodeAddress The address of this node. Set to 0 to only receive
   * packets.
   */
  Network_Interface(uint16_t nodeAddress);

  /**
   * @brief Set the address of this node.
   *
   * @param nodeAddress The address of this node.
   */
  void setNodeAddress(uint16_t nodeAddress);

  /**
   * @brief Get the address of this node.
   *
   * @return uint16_t The address of this node.
   */
  uint16_t getNodeAddress() const;

  /**
   * @brief Publish a network packet to be sent to the destination.
   *
   * @param packet The packet to be sent.
   */
  virtual void sendPacket(const NetworkPacket &packet) = 0;

  /**
   * @brief Adds a handler to be called when a network packet is received.
   * @param handler The handler function to be added.
   */
  void addPacketReceiveHandler(
      Core::HandlerGroup<const NetworkPacket &>::HandlerFunction handler);

  /**
   * @brief Clears all handlers that are called when a network packet is
   * received.
   */
  void clearPacketReceiveHandlers();
};

} // namespace Net

} // namespace VCTR

#endif