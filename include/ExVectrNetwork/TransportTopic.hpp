#ifndef EXVECTRNETWORK_TRANSPORTTOPIC_H_
#define EXVECTRNETWORK_TRANSPORTTOPIC_H_

#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrNetwork/TransportCallback.hpp"

namespace VCTR {

namespace Net {

/**
 * @brief   The topic transport system can be used to link topics together over
 * a network. Enabling easy communication between different systems.
 * @note    This does not guarantee delivery of messages. Also be aware of the
 * datarate of the datalink/physical layer and buffering limits.
 * @tparam  TYPE The type of data to be sent over the network.
 */
template <typename TYPE> class TransportTopic {
private:
  /// @brief The destination address to send data to.
  uint16_t dstAddress_ = 0;

  /// @brief If true then this topic will not receive data (Send only).
  bool noReceive_ = false;

  /// @brief The transport layer to use for sending and receiving data.
  TransportCallback transport_;

  /// @brief Basically receives data from the transport layer
  Core::Callback_Subscriber<TransportCallback::TransportData,
                            TransportTopic<TYPE>>
      transportSubr_;

  /// @brief Communication with the topic
  Core::Callback_Subscriber<TYPE, TransportTopic<TYPE>> topicSubr_;

public:
  /**
   * @param channel The port to use for this topic for bidirectional
   * communication. Only a single topic can be used per channel. (Theoretically
   * multiple topics can be used on the same port if the data length is
   * different.)
   * @param dstAddress The destination address to send data to.
   * @param noReceive If true then this topic will not receive data (Send only).
   */
  TransportTopic(uint16_t channel, uint16_t dstAddress, bool noReceive = false)
      : dstAddress_(dstAddress), transport_(channel) {
    transportSubr_.subscribe(transport_.getReceiveTopic());
    transportSubr_.setCallback(this, &TransportTopic::receiveData);
    topicSubr_.setCallback(this, &TransportTopic::sendData);
  }

  /**
   * @param channel The port to use for this topic for bidirectional
   * communication. Only a single topic can be used per channel. (Theoretically
   * multiple topics can be used on the same port if the data length is
   * different.)\
   * @param dstAddress The destination address to send data to.
   * @param node The network node to use for sending and receiving data.
   * @param noReceive If true then this topic will not receive data (Send only).
   */
  TransportTopic(uint16_t channel, uint16_t dstAddress, Network_Interface &node,
                 bool noReceive = false)
      : TransportTopic(channel, dstAddress, noReceive) {
    setNetworkNode(node);
  }

  /**
   * @param channel The port to use for this topic for bidirectional
   * communication. Only a single topic can be used per channel. (Theoretically
   * multiple topics can be used on the same port if the data length is
   * different.)
   * @param dstAddress The destination address to send data to.
   * @param node The network node to use for sending and receiving data.
   * @param topic The topic to use for sending and receiving data.
   * @param noReceive If true then this topic will not receive data (Send only).
   */
  TransportTopic(uint16_t channel, uint16_t dstAddress, Network_Interface &node,
                 Core::Topic<TYPE> &topic, bool noReceive = false)
      : TransportTopic(channel, dstAddress, node, noReceive) {
    setTopic(topic);
  }

  /**
   * @brief   Set the channel (port) to use for sending and receiving packets.
   * @param channel The channel to use.
   */
  void setChannel(uint16_t channel) { transport_.setPort(channel); }

  /**
   * @brief   Set the destination address to receive our packets.
   * @param dstAddress The destination address to use.
   */
  void setDstAddress(uint16_t dstAddress) { dstAddress_ = dstAddress; }

  /**
   * @brief   Sets the flag to not receive data.
   * @param noReceive If true then this topic will not receive data (Send only).
   */
  void setNoReceive(bool noReceive = true) { noReceive_ = noReceive; }

  /**
   * @brief   Set the network node to use for sending and receiving packets.
   * @param node The network node to use.
   */
  void setNetworkNode(Network_Interface &node) {
    transport_.setNetworkNode(node);
  }

  /**
   * @brief   Sends the given data to the given address and port.
   * @param data The data to send and the destination address and port. The
   * source address and port will be added automatically.
   */
  void setTopic(Core::Topic<TYPE> &topic) { topicSubr_.subscribe(topic); }

private:
  /**
   * @brief   Callback for receiving data from the transport layer.
   * @param data The data received from the transport layer.
   */
  void receiveData(const Net::TransportCallback::TransportData &data) {

    if (noReceive_ || data.srcPort != transport_.getPort() ||
        data.data.size() != sizeof(TYPE))
      return;

    TYPE item;
    for (size_t i = 0; i < sizeof(TYPE); i++)
      ((uint8_t *)&item)[i] = data.data[i];
    topicSubr_.publish(item);
  }

  /**
   * @brief   Sends the given data to the given address and port using the
   * transport layer.
   * @param data The data to send and the destination address and port. The
   * source address and port will be added automatically.
   */
  void sendData(const TYPE &data) {

    TransportCallback::TransportData dataToSend;
    dataToSend.dstAddress = dstAddress_;
    dataToSend.dstPort = transport_.getPort();

    for (size_t i = 0; i < sizeof(TYPE); i++)
      dataToSend.data.append(((uint8_t *)&data)[i]);

    transport_.send(dataToSend);
  }
};

} // namespace Net

} // namespace VCTR

#endif