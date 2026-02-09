#include "ExVectrCore/cyclic_checksum.hpp"
#include "ExVectrCore/list.hpp"
#include "ExVectrCore/list_static.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrNetwork/Datalink.hpp"
#include "ExVectrNetwork/NetworkNode.hpp"
#include "ExVectrNetwork/structs/NetworkPacket.hpp"

namespace VCTR {

namespace Net {

NetworkNode::NetworkNode(uint16_t nodeAddress, int64_t disconnectTimeout)
    : Network_Interface(nodeAddress),
      Task_Periodic("NetworkNode", 100 * Core::MILLISECONDS) // 10Hz
{
  timeoutInterval_ = disconnectTimeout;
  sendInterval_ =
      timeoutInterval_ /
      10; // Send a heartbeat packet every 1/5 of the timeout interval.

  Core::getSystemScheduler().addTask(*this);
}

NetworkNode::NetworkNode(uint16_t nodeAddress, Datalink_Interface &datalink,
                         int64_t disconnectTimeout)
    : NetworkNode(nodeAddress, disconnectTimeout) {
  addDatalink(datalink);
}

// NetworkNode::~NetworkNode()
//{
// }

void NetworkNode::addDatalink(Datalink_Interface &datalink) {
  datalinks_.append(&datalink);
  datalink.addReceiveHandler(
      [this](const Dataframe &dataframe) { receivePacket(dataframe.data); });
}

bool NetworkNode::isNodeReachable(uint16_t nodeAddress) {
  for (size_t i = 0; i < nodeList_.size(); i++) {
    if (nodeList_[i].nodeAddress == nodeAddress) {
      return true;
    }
  }
  return false;
}

void NetworkNode::taskThread() {

  if (Core::NOW() - lastSend_ > sendInterval_) {
    lastSend_ = Core::NOW();
    NetworkPacket packet;
    packet.type = NetworkPacketType::HEARTBEAT;
    packet.hops = 0;
    packet.dstAddress = 0xFFFF; // Broadcast to all nodes.
    packet.srcAddress = nodeAddress_;
    packet.payload.placeBack(0); // Empty payload.
    sendPacket(packet);
  }

  // Check if any nodes are unreachable
  for (size_t i = 0; i < nodeList_.size(); i++) {
    if (Core::NOW() - nodeList_[i].lastSeen > timeoutInterval_) {
      nodeList_.removeAtIndex(i);
      i--;
    }
  }
}

void NetworkNode::sendPacket(const NetworkPacket &packet) {

  VRBS_MSG("Sending Packet! Pointer: %d\n", this);

  if (packet.payload.size() == 0) {
    LOG_MSG("Packet empty! \n");
    return;
  }

  auto packetSend = packet;
  packetSend.srcAddress = nodeAddress_;
  // packetSend.type = NetworkPacketType::DATA;

  if (nodeAddress_ ==
      packetSend.dstAddress) { // If this packet is for this node, publish it
                               // directly to the receive topic.
    packetReceiveHandlers_.callHandlers(packet);
    return;
  }

  // LOG_MSG("Packet input size: %d, Packet copy size: %d\n",
  // packet.payload.size(), packetSend.payload.size());

  Core::ListArray<uint8_t> packetBytes;
  packetBytes.setSize(packet.payload.size() + 8);
  if (!packPacket(packetSend, packetBytes)) {
    LOG_MSG("Failed to pack packet! \n");
    return;
  }

  for (size_t i = 0; i < datalinks_.size(); i++) {
    Dataframe dataframe;
    for (size_t j = 0; j < packetBytes.size(); j++)
      dataframe.data.placeBack(packetBytes[j]);
    datalinks_[i]->transmitDataframe(dataframe);
  }
  lastSend_ = Core::NOW(); // Update the last send time.
}

void NetworkNode::receivePacket(const Core::List<uint8_t> &data) {

  VRBS_MSG("Received a packet. Size: %d Pointer: %d\n", data.size(), this);

  // Convert the data to a packet
  NetworkPacket packet;
  if (!unpackPacket(packet, data)) {
    VRBS_MSG("Failed to unpack packet! \n");
    return;
  }

  // Update the reachable nodes list
  bool found = false;
  for (size_t i = 0; i < nodeList_.size(); i++) {
    if (nodeList_[i].nodeAddress == packet.srcAddress) {
      nodeList_[i].lastSeen = Core::NOW();
      found = true;
      break;
    }
  }
  if (!found) // // If the node is not in the list, add it
  {
    NodeInfo nodeInfo;
    nodeInfo.nodeAddress = packet.srcAddress;
    nodeInfo.lastSeen = Core::NOW();
    nodeList_.appendIfNotInListArray(nodeInfo);
  }

  if (packet.type == NetworkPacketType::HEARTBEAT) {
    return; // Ignore heartbeat packets.
  }

  // Check if the node is for us and publish it to the receive topic
  if (packet.dstAddress == nodeAddress_ ||
      packet.dstAddress ==
          UINT16_MAX) // If this packet is for this node, publish it directly to
                      // the receive topic.
  {
    // Decrement hops if not zero
    if (packet.hops > 0)
      packet.hops--;

    // Send packet to receive topic (Network -> Transport)
    packetReceiveHandlers_.callHandlers(packet);
  }
}

bool NetworkNode::unpackPacket(NetworkPacket &packet,
                               const Core::List<uint8_t> &data) {

  VRBS_MSG("Unpacking packet. Length: %d.\n", data.size());

  packet.type = NetworkPacketType(data[0]);
  packet.hops = data[1];
  packet.dstAddress = (data[2] << 8) | data[3];
  packet.srcAddress = (data[4] << 8) | data[5];
  // Byte 6 is checksum
  uint8_t payloadSize = data[7];

  VRBS_MSG("Packet type: %d, Hops: %d, Dst: %d, Src: %d, Payload size: %d.\n",
           packet.type, packet.hops, packet.dstAddress, packet.srcAddress,
           payloadSize);

  if (data.size() != size_t(payloadSize + 8)) {
    LOG_MSG("Data buffer wrong size! Packet size: %d, Data size: %d \n",
            packet.payload.size() + 8, data.size());
    return false;
  }

  packet.payload.clear();
  for (size_t i = 0; i < payloadSize; i++) {
    packet.payload.placeBack(data[8 + i]);
  }

  // Calculate checksum
  uint8_t checksum = 0; // Core::computeCrc(data, 0);
  for (size_t i = 0; i < data.size(); i++) {
    if (i == 6) // Skip checksum byte
      continue;
    checksum += data[i];
  }

  if (checksum != data[6]) {
    LOG_MSG("Checksum failed! Expected: %d, Is: %d \n", data[6], checksum);
    return false;
  }

  return true;
}

bool NetworkNode::packPacket(const NetworkPacket &packet,
                             Core::List<uint8_t> &data) {

  VRBS_MSG("Packing packet. Payload len: %d.\n", packet.payload.size());
  if (data.size() < packet.payload.size() + 8) {
    LOG_MSG("Data buffer too small! \n");
    return false;
  }

  data[0] = uint8_t(packet.type);
  data[1] = packet.hops;
  data[2] = packet.dstAddress >> 8;
  data[3] = packet.dstAddress & 0xFF;
  data[4] = packet.srcAddress >> 8;
  data[5] = packet.srcAddress & 0xFF;
  // Byte 6 is checksum
  data[6] = 0;
  data[7] = packet.payload.size();

  for (size_t i = 0; i < packet.payload.size(); i++) {
    data[8 + i] = packet.payload[i];
  }

  // Calculate checksum
  uint8_t checksum = 0; // Core::computeCrc(data, 0);
  for (size_t i = 0; i < data.size(); i++) {
    checksum += data[i];
  }

  data[6] = checksum;

  return true;
}

} // namespace Net

} // namespace VCTR