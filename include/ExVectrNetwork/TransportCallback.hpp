#ifndef EXVECTRNETWORK_TRANSPORTCALLBACK_H_
#define EXVECTRNETWORK_TRANSPORTCALLBACK_H_

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrNetwork/interfaces/NetworkInterface.hpp"

namespace VCTR {

namespace Net {

/**
 * @brief   The simple and fast transport class. Similar to UDP it will break up
 * large data and send it in segments, rebuilding them on the other end and
 * checking for corruption. But does not guarantee delivery.
 *          This is mainly used as a basis for other transport methods, but can
 * be used as is.\
 * @note    Due to the underlying systems also being based on callbacks, attempt
 * to reduce the further number of callbacks to offload the datalink layer (Head
 * of the chain of callbacks).
 */
class TransportCallback {
private:
  /// @brief The structure used for sending and receiving data.
  struct TransportData {
    uint16_t srcAddress;
    uint16_t srcPort;
    uint16_t dstAddress;
    uint16_t dstPort;
    /// @brief The data being sent or received. The creator of this structure is
    /// responsible for managing the memory.
    Core::ListArray<uint8_t> data;
  };

  /// @brief A version control for the transport simple. This is added together
  /// with the transport identifier to not conflict with other transport
  /// protocols.
  static constexpr uint8_t transportSimpleVersion = 2;
  static constexpr uint8_t transportSimpleID = 1;

  /// @brief The limit of segments that can be stored in the segment buffer.
  // static constexpr uint16_t segmentBufferLimit = 100;

  /// @brief The size limit of each segment in bytes.
  static constexpr uint8_t segmentSize = 128;

  Network_Interface *netNode_ = nullptr;

  /// @brief The port that this transport is using.
  uint16_t port_ = 0;

  /// @brief The ID of the current data being sent. Must always be different
  /// than last.
  uint8_t sendingID_ = 1;

  /// @brief The ID of the current data being received.
  uint8_t rcvID_ = 0;
  /// @brief The number of segments that are expected to be received.
  uint16_t numSegments_ = 0;
  /// @brief The number of bytes that are expected to be received.
  uint16_t numBytes_ = 0;
  /// @brief The current number of segments received.
  uint16_t curSegment_ = 0;
  /// @brief The expected checksum of the data being received.
  uint8_t checksum_ = 0;

  /// @brief A buffer to store the data being received until all segments are
  /// received.
  Core::ListArray<NetworkPacket> segmentBuffer_;

  /// @brief Buffer containing the final received data.
  Core::ListArray<uint8_t> receivedData_;

  /// @brief The topic to receive data transmitting from the other end.
  Core::Topic<TransportData> receiveTopic_;
  /// @brief The topic to transmit data to the other end.
  Core::Topic<TransportData> transmitTopic_;
  Core::Callback_Subscriber<TransportData, TransportCallback>
      transmitTopicSubr_;

  Core::HandlerGroup<const NetworkPacket &> receivePacketHandlers_;

public:
  /**
   * @brief Construct a new TransportSimple object.
   * @param port The port to use for this transport.
   */
  TransportCallback(uint16_t port);

  /**
   * @brief Construct a new TransportSimple object.
   * @param port The port to use for this transport.
   * @param node The network node to use for sending and receiving packets.
   */
  TransportCallback(uint16_t port, Network_Interface &node);

  /**
   * @brief   Set the network node to use for sending and receiving packets.
   * @param node The network node to use.
   */
  void setNetworkNode(Network_Interface &node);

  /**
   * @brief   Set the port to use for sending and receiving packets.
   * @param port The port to use.
   */
  void setPort(uint16_t port);
  uint16_t getPort();

  /**
   * @brief   Sends the given data to the given address and port.
   * @param data The data to send.
   * @param dstAddress The destination address to send data to.
   * @param dstPort The destination port to send data to.
   */
  void send(const Core::List<uint8_t> &data, uint16_t dstAddress,
            uint16_t dstPort);

  /**
   * @brief   Gets the topic to receive data from the network node. Subscribe to
   * this topic to receive data from the other end.
   * @returns The topic to receive data from the network node.
   */
  Core::Topic<TransportData> &getReceiveTopic();

  /**
   * @brief   Gets the topic to transmit data to the network node. Publish to
   * this topic to send data to the other end.
   * @returns The topic to transmit data to the network node.
   */
  Core::Topic<TransportData> &getTransmitTopic();

private:
  /**
   * @brief   Sends a segment of data to the given address and port. Basically
   * appends the required information to the data and sends it.
   * @param data The data to send.
   * @param dstAddress The destination address.
   * @param dstPort The destination port.
   * @param segmentID The ID of the segment being sent.
   */
  void sendSegment(NetworkPacket &segment, uint16_t order, uint16_t dstAddress,
                   uint16_t dstPort, uint8_t id);

  /**
   * @brief   Callback function for receiving data from the network node.
   * @param packet The packet that was received.
   */
  void receivePacketCallback(const NetworkPacket &packet);
};

} // namespace Net

} // namespace VCTR

#endif