#include "ExVectrCore/cyclic_checksum.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrNetwork/TransportCallback.hpp"
#include "ExVectrNetwork/interfaces/NetworkInterface.hpp"

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

namespace VCTR {

namespace Net {

TransportCallback::TransportCallback(uint16_t port) : port_(port) {}

TransportCallback::TransportCallback(uint16_t port, Network_Interface &node)
    : TransportCallback(port) {
  setNetworkNode(node);
}

/**
 * @brief   Set the network node to use for sending and receiving packets.
 * @param node The network node to use.
 */
void TransportCallback::setNetworkNode(Network_Interface &node) {
  netNode_ = &node;
  netNode_->addPacketReceiveHandler(
      [this](const NetworkPacket &packet) { receivePacketCallback(packet); });
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
  NetworkPacket packet;
  // packet.dstAddress = data.dstAddress;
  // packet.srcAddress = port_;

  packet.payload.placeBack(numSegments >> 8);
  packet.payload.placeBack(numSegments & 0xFF);
  packet.payload.placeBack(numBytes >> 8);
  packet.payload.placeBack(numBytes & 0xFF);
  packet.payload.placeBack(crc);

  // Send the first packet
  sendSegment(packet, 0, dstAddress, dstPort, sendingID_);

  // Send the rest of the packets
  for (uint16_t i = 0; i < numSegments; i++) {

    packet.payload.clear();

    for (uint16_t j = 0; j < segmentSize; j++) {
      if (i * segmentSize + j >= numBytes) {
        break;
      }
      packet.payload.placeBack(data[i * segmentSize + j]);
    }

    VRBS_MSG("Sending data segment %d. \n", i + 1);

    sendSegment(packet, i + 1, dstAddress, dstPort, sendingID_);
  }

  // Increment the sending ID
  sendingID_++;
}

void TransportCallback::sendSegment(NetworkPacket &segment, uint16_t order,
                                    uint16_t dstAddress, uint16_t dstPort,
                                    uint8_t id) {

  // Add the destination address.
  segment.dstAddress = dstAddress;

  // Add segment data [srcPortHigh, srcPortLow, dstPortHigh, dstPortLow, ID,
  // transportIdentifier]
  segment.payload.placeBack(port_ >> 8);
  segment.payload.placeBack(port_ & 0xFF);
  segment.payload.placeBack(dstPort >> 8);
  segment.payload.placeBack(dstPort & 0xFF);
  segment.payload.placeBack(order >> 8);
  segment.payload.placeBack(order & 0xFF);
  segment.payload.placeBack(id);
  segment.payload.placeBack(transportSimpleVersion + transportSimpleID);

  netNode_->sendPacket(segment);
}

/**
 * @brief   Callback function for receiving data from the network node.
 * @param packet The packet that was received.
 */
void TransportCallback::receivePacketCallback(const NetworkPacket &packet) {

  VRBS_MSG("Received packet. len %d. \n", packet.payload.size());

  if (packet.payload.size() < 8) { // Packet is too small. Discard.
    LOG_MSG("Received packet is too small. Its %d bytes long \n",
            packet.payload.size());
    return;
  }

  // Check if packet is a transport packet and correct version
  uint8_t identifier = packet.payload(-1);
  if (identifier != transportSimpleVersion + transportSimpleID) {
    LOG_MSG("Received packet is not a transport packet. Identifier: %d. \n",
            int(identifier));
    return;
  }

  // Unpack the segment info appended to the end of the packet
  uint16_t srcPort = (packet.payload(-8) << 8) | packet.payload(-7);
  uint16_t dstPort = (packet.payload(-6) << 8) | packet.payload(-5);
  uint16_t order = (packet.payload(-4) << 8) | packet.payload(-3);
  uint8_t id = packet.payload(-2);

  // Check if packet is for this port, if not return
  if (dstPort != port_) {
    VRBS_MSG("Received transport packet for a different port. \n");
    return;
  }

  VRBS_MSG("Received transport packet for this port. \n");

  if (id !=
      rcvID_) { // We are receiving new data. Discard old data and restart.

    uint16_t numSegments = (packet.payload[0] << 8) | packet.payload[1];
    uint16_t numBytes = (packet.payload[2] << 8) | packet.payload[3];
    uint16_t checksum = packet.payload[4];

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

    segmentBuffer_.clear();
    curSegment_ = 0;

    numSegments_ = numSegments;
    numBytes_ = numBytes;
    checksum_ = checksum;
    rcvID_ = rcvID;

    VRBS_MSG("Received new data. Segments: %d, Bytes: %d, Checksum: %d. \n",
             numSegments_, numBytes_, checksum_);

    return;
  }
  VRBS_MSG("Received data segment %d. \n", order);

  // Place the segment into the buffer.
  segmentBuffer_.append(packet);
  curSegment_++;

  // Check if all segments are received
  if (curSegment_ == numSegments_) {

    // All segments are received. Reconstruct the data.
    VRBS_MSG("All segments received. \n");

    receivedData_.clear();
    TransportData data;

    // Reconstruct the data by placing each segments data from segmentBuffer_ in
    // the correct order inside the receivedData_ buffer.
    for (uint16_t curSeg = 1; curSeg <= numSegments_; curSeg++) {

      // LOG_MSG("Reconstructing segment %d. \n", curSeg);

      // Find the next segment
      for (size_t i = 0; i < segmentBuffer_.size(); i++) {
        NetworkPacket &segment = segmentBuffer_[i];
        // uint16_t order = (segment.payload(-4) << 8) | segment.payload(-3);
        // LOG_MSG("Segment %d. \n", order);
        if (order == curSeg) {
          // LOG_MSG("Found segment %d. \n", order);
          //  Place the data into the receivedData_ buffer
          for (size_t j = 0; j < segment.payload.size() - 8; j++) {
            data.data.append(segment.payload[j]);
          }
          break;
        }
      }
    }

    // Check if the data is correct
    uint8_t crc = Core::computeCrc(data.data, 0);
    if (crc != checksum_ ||
        numBytes_ != data.data.size()) { // Data is corrupt. Discard.

      LOG_MSG("Received data is corrupt. CRC rcv: %d, Expected: %d. Data "
              "length: %d. \n",
              crc, checksum_, data.data.size());
      receivedData_.clear();
    } else { // publish the data

      VRBS_MSG("Received data is correct. CRC rcv: %d, Expected: %d. Data "
               "length: %d. \n",
               crc, checksum_, data.data.size());

      data.dstAddress = packet.dstAddress;
      data.dstPort = dstPort;
      data.srcAddress = packet.srcAddress;
      data.srcPort = srcPort;
      receiveTopic_.publish(data);
    }

    // Clear the segment buffer
    segmentBuffer_.clear();
    checksum_ = curSegment_ = numBytes_ = numSegments_ = 0;
  }
}

} // namespace Net

} // namespace VCTR
