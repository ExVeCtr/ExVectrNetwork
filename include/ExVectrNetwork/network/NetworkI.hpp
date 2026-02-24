#ifndef EXVECTRNETWORK_INTERFACES_NETWORKINTERFACE_HPP_
#define EXVECTRNETWORK_INTERFACES_NETWORKINTERFACE_HPP_

#include "ExVectrCore/handler.hpp"

#include "ExVectrNetwork/datalink/DatalinkI.hpp"
#include "ExVectrNetwork/network/NetworkHeader.hpp"

namespace VCTR::network::network {

/**
 * @brief Network layer class. This class takes care of routing packets to their
 * destination.
 */
class NetworkI {
protected:
  /// The address of this node.
  uint16_t nodeAddress_;

  Core::HandlerGroup<const NetworkPacketHeader &,
                     const Core::ListArray<uint8_t> &>
      packetReceiveHandlers_;

public:
  /**
   * @brief Construct a new Network Node object.
   *
   * @param nodeAddress The address of this node. Set to 0 to only receive
   * packets.
   */
  NetworkI(uint16_t nodeAddress);

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
  virtual void sendPacket(const NetworkPacketHeader &header,
                          const Core::ListArray<uint8_t> &payload = 0) = 0;

  /**
   * @brief Get the maximum packet size that can be transmitted by the network.
   * @note packets over this size will be dropped and not transmitted.
   * @return size_t The maximum packet size in bytes.
   */
  virtual size_t getMaxPacketSize() const = 0;

  /**
   * @brief Adds a handler to be called when a network packet is received.
   * @param handler The handler function to be added.
   */
  void addPacketReceiveHandler(
      Core::HandlerGroup<const NetworkPacketHeader &,
                         const Core::ListArray<uint8_t> &>::HandlerFunction
          handler);

  /**
   * @brief Clears all handlers that are called when a network packet is
   * received.
   */
  void clearPacketReceiveHandlers();
};

} // namespace VCTR::network::network

#endif