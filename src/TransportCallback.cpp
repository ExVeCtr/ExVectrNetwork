#include "ExVectrCore/cyclic_checksum.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrNetwork/network/NetworkI.hpp"
#include "ExVectrNetwork/transport/TransportCallback.hpp"

/**
 * The transport simple works by breaking up the data into segments and sending
 * them immediately. The initial packet is small and only contains the new
 * data's number of segments, bytes, checksum and ID. The following
 * packets/segments contain the data. The receiving end will store all data in a
 * buffer until all segments are received. Once all segments are received, the
 * data is checked for corruption and then forwarded.
 *
 * Specifically each packet MUST append the following 8 bytes: [srcPortHigh,
 * srcPortLow, dstPortHigh, dstPortLow, orderHigh, orderLow, ID,
 * transportIdentifier] A packet with order 0 is the info packet and contains
 * the number of segments, bytes, checksum and ID.
 */

namespace VCTR::network::transport {

TransportCallback::TransportCallback(uint16_t port) : port_(port) {}

TransportCallback::TransportCallback(uint16_t port,
                                     VCTR::network::network::NetworkI &node)
    : TransportCallback(port) {
  setNetworkNode(node);
}

/**
 * @brief   Set the network node to use for sending and receiving packets.
 * @param node The network node to use.
 */
void TransportCallback::setNetworkNode(VCTR::network::network::NetworkI &node) {
  netNode_ = &node;
  netNode_->addPacketReceiveHandler(
      [this](const VCTR::network::network::NetworkPacketHeader &header,
             const Core::ListArray<uint8_t> &payload) {
        receivePacketCallback(header, payload);
      });
}

void TransportCallback::setPort(uint16_t port) { port_ = port; }
uint16_t TransportCallback::getPort() { return port_; }

/**
 * @brief   Sends the given data to the given address and port.
 * @param data The data to send and the destination address and port. The source
 * address and port will be added automatically.
 */
void TransportCallback::send(const Core::List<uint8_t> &data,
                             uint16_t dstAddress, uint16_t dstPort) {

  if (data.size() == 0) {
    LOG_MSG("Data is null. \n");
    return;
  }

  // Calculate number of segments
  uint16_t numBytes = data.size();
  uint16_t numSegments = numBytes / segmentSize;
  uint16_t finalSegmentSize = numBytes % segmentSize;
  if (finalSegmentSize > 0) {
    numSegments++;
  }

  // Calculate checksum of data.
  uint8_t crc = Core::computeCrc(data, 0);

  VRBS_MSG("Sending info segment. Segments: %d, Bytes: %d, Checksum: %d. \n",
           numSegments, numBytes, crc);

  // Create the first packet (Info packet)
  VCTR::network::network::NetworkPacketHeader header;
  header.type = VCTR::network::network::NetworkPacketType::DATA;
  header.srcAddress = netNode_->getNodeAddress();
  header.dstAddress = dstAddress;
  header.hops = 1;

  Core::ListArray<uint8_t> packet;
  packet.append(numSegments >> 8);
  packet.append(numSegments & 0xFF);
  packet.append(numBytes >> 8);
  packet.append(numBytes & 0xFF);
  packet.append(crc);
  // Send the first packet
  sendSegment(header, packet, 0, dstPort, sendingID_);

  // Send the rest of the packets
  for (uint16_t i = 0; i < numSegments; i++) {

    packet.clear();

    for (uint16_t j = 0; j < segmentSize; j++) {
      if (i * segmentSize + j >= numBytes) {
        break;
      }
      packet.append(data[i * segmentSize + j]);
    }

    VRBS_MSG("Sending data segment %d. \n", i + 1);

    sendSegment(header, packet, i + 1, dstPort, sendingID_);
  }

  // Increment the sending ID
  sendingID_++;
}

void TransportCallback::sendSegment(
    const VCTR::network::network::NetworkPacketHeader &header,
    Core::ListArray<uint8_t> &data, uint16_t order, uint16_t dstPort,
    uint8_t id) {

  data.append(port_ >> 8);
  data.append(port_ & 0xFF);
  data.append(dstPort >> 8);
  data.append(dstPort & 0xFF);
  data.append(order >> 8);
  data.append(order & 0xFF);
  data.append(id);
  data.append(transportSimpleVersion + transportSimpleID);

  netNode_->sendPacket(header, data);
}

/**
 * @brief   Callback function for receiving data from the network node.
 * @param packet The packet that was received.
 */
void TransportCallback::receivePacketCallback(
    const VCTR::network::network::NetworkPacketHeader &header,
    const Core::ListArray<uint8_t> &payload) {

  if (payload.size() < 8) { // Packet is too small. Discard.
    LOG_MSG("Received packet is too small. Its %d bytes long \n",
            payload.size());
    return;
  }

  // Check if packet is a transport packet and correct version
  uint8_t identifier = payload(-1);
  if (identifier != transportSimpleVersion + transportSimpleID) {
    LOG_MSG("Received packet is not a transport packet. Identifier: %d. \n",
            int(identifier));
    return;
  }

  // Unpack the segment info appended to the end of the packet
  uint16_t srcPort = (payload(-8) << 8) | payload(-7);
  uint16_t dstPort = (payload(-6) << 8) | payload(-5);
  uint16_t order = (payload(-4) << 8) | payload(-3);
  uint8_t id = payload(-2);

  // Check if packet is for this port, if not return
  if (dstPort != port_) {
    return;
  }

  if (id !=
      rcvID_) { // We are receiving new data. Discard old data and restart.

    uint16_t numSegments = (payload[0] << 8) | payload[1];
    uint16_t numBytes = (payload[2] << 8) | payload[3];
    uint16_t checksum = payload[4];

    uint8_t rcvID = id;

    if (numSegments == 0 || numBytes == 0 ||
        numBytes < numSegments) { // No data to receive or something is wrong
                                  // with the data. Discard.
      LOG_MSG("Received something wierd. Will be discarded. ID: %d, Segments: "
              "%d, Bytes: %d, Checksum: %d. \n",
              rcvID, numSegments, numBytes, checksum);
      // segmentBuffer_.clear();
      // curSegment_ = 0;
      // numSegments_ = numBytes_ = checksum_ = 0;
      return;
    }

    curSegment_ = 0;

    numSegments_ = numSegments;
    numBytes_ = numBytes;
    checksum_ = checksum;
    rcvID_ = rcvID;

    if (receivedData_.size() < numBytes_) {
      receivedData_.setSize(numBytes_);
    }

    VRBS_MSG("Received new data. Segments: %d, Bytes: %d, Checksum: %d. \n",
             numSegments_, numBytes_, checksum_);

    return;
  }
  VRBS_MSG("Received data segment %d. \n", order);

  // Place the segment into the buffer.
  for (size_t i = order * segmentSize;
       i < (order + 1) * segmentSize && i < numBytes_; i++) {
    receivedData_[i] = payload[i - order * segmentSize];
  }
  curSegment_++;

  // Check if all segments are received
  if (curSegment_ == numSegments_) {

    // All segments are received. Reconstruct the data.
    VRBS_MSG("All segments received. \n");

    receivedData_.clear();
    TransportData data;

    // Check if the data is correct
    uint8_t crc = Core::computeCrc(receivedData_, 0);
    if (crc != checksum_ ||
        numBytes_ != receivedData_.size()) { // Data is corrupt. Discard.

      LOG_MSG("Received data is corrupt. CRC rcv: %d, Expected: %d. Data "
              "length: %d. \n",
              crc, checksum_, receivedData_.size());
      receivedData_.clear();
    } else { // publish the data

      data.dstAddress = header.dstAddress;
      data.dstPort = dstPort;
      data.srcAddress = header.srcAddress;
      data.srcPort = srcPort;
      receiveTopic_.publish(data);
    }

    checksum_ = curSegment_ = numBytes_ = numSegments_ = 0;
  }
}

} // namespace VCTR::network::transport
